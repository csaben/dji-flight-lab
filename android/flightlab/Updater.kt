package dji.sampleV5.aircraft.flightlab

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.os.Build
import android.provider.Settings
import androidx.core.content.FileProvider
import androidx.core.content.pm.PackageInfoCompat
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONObject
import java.io.File
import java.util.concurrent.TimeUnit

/**
 * Self-update from cradle over the tailnet. The publish script writes dl/version.json next to the APK:
 *   {"versionCode": 451234, "versionName": "2026.11.08-0612", "apk": "flightlab.apk", "size": 255000000, "notes": "..."}
 * The APK is signed with the same key every build, so installing it updates in place and keeps settings.
 */
class Updater(private val act: Activity) {

    /** mirror = the Mac's direct-connection copy (fast), null if the build has none. */
    data class Remote(val code: Long, val name: String, val url: String, val size: Long, val notes: String, val mirror: String?)

    private val http = OkHttpClient.Builder().connectTimeout(8, TimeUnit.SECONDS).readTimeout(60, TimeUnit.SECONDS).build()
    @Volatile var busy = false; private set

    val localCode: Long get() = PackageInfoCompat.getLongVersionCode(act.packageManager.getPackageInfo(act.packageName, 0))
    val localName: String get() = act.packageManager.getPackageInfo(act.packageName, 0).versionName ?: "?"

    /** Blocking; call off the main thread. Asks cradle, then the Mac mirror. Null if neither answers. */
    fun check(): Remote? = fetch(BASE) ?: fetch(MIRROR_BASE)

    private fun fetch(base: String): Remote? = try {
        http.newCall(Request.Builder().url(base + "version.json?t=" + System.currentTimeMillis()).build()).execute().use { r ->
            if (!r.isSuccessful) return null
            val j = JSONObject(r.body!!.string())
            val apk = j.optString("apk", "flightlab.apk")
            val mirror = j.optJSONArray("mirrors")?.optJSONObject(0)?.optString("url")?.takeIf { it.isNotBlank() }
            Remote(j.getLong("versionCode"), j.optString("versionName"), if (apk.startsWith("http")) apk else BASE + apk, j.optLong("size"), j.optString("notes"), mirror)
        }
    } catch (_: Exception) { null }

    /** Blocking download with progress 0..1, then hands the APK to the system installer. Returns an error or null. */
    fun downloadAndInstall(r: Remote, url: String, progress: (Double) -> Unit): String? {
        if (busy) return "Already downloading."
        busy = true
        try {
            val dir = act.getExternalFilesDir(null) ?: return "No storage."
            val out = File(dir, "update.apk"); val tmp = File(dir, "update.apk.part")
            http.newCall(Request.Builder().url(url).build()).execute().use { resp ->
                if (!resp.isSuccessful) return "Download failed: HTTP ${resp.code}"
                val body = resp.body!!
                val total = body.contentLength().takeIf { it > 0 } ?: r.size
                body.byteStream().use { inp ->
                    tmp.outputStream().use { o ->
                        val buf = ByteArray(256 * 1024); var n: Int; var got = 0L; var lastP = -1.0
                        while (inp.read(buf).also { n = it } >= 0) {
                            o.write(buf, 0, n); got += n
                            if (total > 0) { val p = got.toDouble() / total; if (p - lastP > 0.01) { lastP = p; progress(p) } }
                        }
                    }
                }
            }
            if (!tmp.renameTo(out)) return "Couldn't save the update."
            act.runOnUiThread { install(out) }
            return null
        } catch (e: Exception) {
            return "Download failed: ${e.message}"
        } finally { busy = false }
    }

    private fun install(apk: File) {
        if (Build.VERSION.SDK_INT >= 26 && !act.packageManager.canRequestPackageInstalls()) {
            // One-time: let Flight Lab install updates. Coming back, tap Update again.
            act.startActivity(Intent(Settings.ACTION_MANAGE_UNKNOWN_APP_SOURCES, Uri.parse("package:" + act.packageName)))
            return
        }
        val uri = FileProvider.getUriForFile(act, act.packageName + ".fileProvider", apk)
        act.startActivity(Intent(Intent.ACTION_VIEW).setDataAndType(uri, "application/vnd.android.package-archive")
            .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_ACTIVITY_NEW_TASK))
    }

    companion object {
        const val BASE = "https://cradle.tailb1b51d.ts.net:8444/dl/"
        const val MIRROR_BASE = "https://cradle-m1.tailb1b51d.ts.net:8990/"
    }
}
