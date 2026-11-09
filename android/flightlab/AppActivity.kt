package dji.sampleV5.aircraft.flightlab

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Color
import android.net.http.SslError
import android.os.Build
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.v5.et.create
import dji.v5.et.get
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.webkit.GeolocationPermissions
import android.webkit.JavascriptInterface
import android.webkit.SslErrorHandler
import android.webkit.WebChromeClient
import android.webkit.WebResourceError
import android.webkit.WebResourceRequest
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.FrameLayout
import android.widget.Toast
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.SDKManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.sampleV5.aircraft.models.MSDKManagerVM
import dji.sampleV5.aircraft.models.globalViewModels
import org.json.JSONObject

/**
 * Flight Lab: the sandbox web app (Plan / Fly / Library / Settings), loaded live from cradle so
 * web tweaks land without reinstalling, on top of the drone's live video, with a JS bridge
 * (window.FL) to the real mission runner. The RC sticks always override, exactly as in the
 * native panel, which stays one tap away (FL.openLab()).
 */
class AppActivity : Activity() {

    private lateinit var web: WebView
    private lateinit var video: SurfaceView
    private lateinit var updater: Updater
    private val ui = Handler(Looper.getMainLooper())
    private var loadFailed = false
    private var pendingGeo: Pair<String, GeolocationPermissions.Callback>? = null
    private val streams get() = MediaDataCenter.getInstance().cameraStreamManager
    private val msdk: MSDKManagerVM by lazy { application.globalViewModels<MSDKManagerVM>().value }
    @Volatile private var sdkInit = "starting"
    @Volatile private var sdkError: String? = null
    @Volatile private var productConnected = false
    @Volatile private var productId = -1

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        if (Build.VERSION.SDK_INT >= 28) window.attributes.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
        updater = Updater(this)
        // SDK state straight from the SDK callbacks (LiveData replays the latest value).
        msdk.lvInitProcess.observeForever { sdkInit = it.first.name }
        msdk.lvRegisterState.observeForever { sdkError = if (it.first) null else (it.second?.toString() ?: "registration failed"); if (!it.first) ui.postDelayed({ sendDiagnostics("register failed") }, 1500) }
        msdk.lvProductConnectionState.observeForever {
            productConnected = it.first; productId = it.second
            if (it.first) { ui.postDelayed({ attachVideo() }, 1000); ui.postDelayed({ attachVideo() }, 4000) }   // drone came up after the surface
        }

        val root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }
        video = SurfaceView(this).apply {
            visibility = View.GONE
            holder.addCallback(object : SurfaceHolder.Callback {
                override fun surfaceCreated(h: SurfaceHolder) {}
                override fun surfaceChanged(h: SurfaceHolder, format: Int, w: Int, hgt: Int) { surf = Triple(h, w, hgt); attachVideo() }
                override fun surfaceDestroyed(h: SurfaceHolder) { surf = null; try { streams.removeCameraStreamSurface(h.surface) } catch (_: Throwable) {} }
            })
        }
        root.addView(video, FrameLayout.LayoutParams(-1, -1))

        WebView.setWebContentsDebuggingEnabled(true)   // chrome://inspect from the Mac
        web = WebView(this).apply {
            setBackgroundColor(Color.TRANSPARENT)
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            settings.mediaPlaybackRequiresUserGesture = false
            settings.setGeolocationEnabled(true)
            settings.cacheMode = WebSettings.LOAD_NO_CACHE   // always the latest sandbox from cradle
            settings.userAgentString = settings.userAgentString + " FlightLabApp/" + updater.localName
            addJavascriptInterface(Bridge(), "FL")
            webViewClient = object : WebViewClient() {
                override fun onReceivedError(view: WebView, req: WebResourceRequest, err: WebResourceError) {
                    if (!req.isForMainFrame) return
                    // circleback (feedback overlay) down? fall back to the planner directly.
                    if (urlIdx < SANDBOX_URLS.size - 1) { urlIdx++; ui.post { loadSandbox() } } else { loadFailed = true; showOffline("${err.description}") }
                }
                override fun onReceivedSslError(view: WebView, handler: SslErrorHandler, error: SslError) { handler.cancel(); showOffline("TLS error") }
                override fun shouldOverrideUrlLoading(view: WebView, req: WebResourceRequest): Boolean {
                    val u = req.url
                    if (u.host == HOST) return false
                    startActivity(Intent(Intent.ACTION_VIEW, u)); return true
                }
            }
            webChromeClient = object : WebChromeClient() {
                override fun onGeolocationPermissionsShowPrompt(origin: String, cb: GeolocationPermissions.Callback) {
                    if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) cb.invoke(origin, true, false)
                    else { pendingGeo = origin to cb; requestPermissions(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION), REQ_LOC) }
                }
            }
        }
        root.addView(web, FrameLayout.LayoutParams(-1, -1))
        setContentView(root)
        immersive()
        loadSandbox()

        val need = arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION).filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (need.isNotEmpty()) requestPermissions(need.toTypedArray(), REQ_LOC)
        ui.postDelayed({ checkForUpdate(manual = false) }, 2500)
        ui.post(waitForSdk)
    }

    /** The drone side starts once the DJI SDK has registered (it initialises in the Application). */
    private val waitForSdk = object : Runnable {
        override fun run() {
            val reg = try { SDKManager.getInstance().isRegistered } catch (_: Throwable) { false }
            if (!reg) { ui.postDelayed(this, 500); return }
            val was = Lab.isReady
            Lab.init(this@AppActivity)
            Lab.onConnection = { up ->
                js("window.flMessage&&flMessage(${JSONObject.quote(if (up) "Drone connected" else "Drone connection lost")})")
                if (!up) { val now = SystemClock.elapsedRealtime(); if (now - lastAutoDiag > 60_000) { lastAutoDiag = now; ui.postDelayed({ sendDiagnostics("auto: drone connection lost") }, 2000) } }
            }
            if (!was && isStarted) Lab.screenStarted()   // count this screen if it's visible now (onStart skipped it)
            pendingAi?.let { applyAi(it) }
            attachVideo()
            js("window.flMessage&&flMessage('Drone link ready')")
        }
    }
    /** (Re)bind the live camera stream to the surface. Safe to call any time; the SDK replaces an existing binding. */
    private var surf: Triple<SurfaceHolder, Int, Int>? = null
    private fun attachVideo() {
        val (h, w, hgt) = surf ?: return
        if (!h.surface.isValid) return
        try {
            streams.putCameraStreamSurface(ComponentIndexType.LEFT_OR_MAIN, h.surface, w, hgt, ICameraStreamManager.ScaleType.CENTER_CROP)
        } catch (t: Throwable) { android.util.Log.w("FlightLab", "video attach failed", t) }
    }
    /** Re-bind the video surface if it's showing but no stream packets arrive (black screen in Real drone mode). */
    private var lastPkts = -1L; private var stalls = 0
    private val videoWatch = object : Runnable {
        override fun run() {
            // Disabled: re-binding the surface on a timer made the picture flicker/black. Kept as a no-op.
        }
    }
    private var pendingAi: String? = null
    private fun applyAi(m: String) { Lab.pilot.mode = when (m) { "off" -> PilotLink.Mode.OFF; "policy" -> PilotLink.Mode.POLICY; else -> PilotLink.Mode.SHADOW } }

    private var isStarted = false
    override fun onStart() { super.onStart(); isStarted = true; if (Lab.isReady) Lab.screenStarted(); Lab.onMessage = { m -> toast(m); js("window.flMessage&&flMessage(${JSONObject.quote(m)})"); autoDiagnostics(m) } }
    override fun onStop() { super.onStop(); isStarted = false; if (Lab.isReady) Lab.screenStopped() }
    override fun onResume() { super.onResume(); immersive(); web.onResume(); js("window.flResume&&flResume()") }
    override fun onPause() { web.onPause(); super.onPause() }
    override fun onWindowFocusChanged(hasFocus: Boolean) { super.onWindowFocusChanged(hasFocus); if (hasFocus) immersive() }

    override fun onDestroy() {
        ui.removeCallbacksAndMessages(null)
        web.destroy()
        super.onDestroy()
    }

    @Deprecated("Deprecated in Java")
    override fun onBackPressed() {
        // Let the sandbox handle back (close a sheet, leave placing); otherwise ignore so a stray
        // swipe can't close the app mid-flight. Use the Flight Lab panel or the home gesture to leave.
        js("window.flBack&&flBack()")
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQ_LOC) pendingGeo?.let { (o, cb) -> cb.invoke(o, grantResults.any { it == PackageManager.PERMISSION_GRANTED }, false); pendingGeo = null }
        if (requestCode == REQ_LOC && grantResults.any { it == PackageManager.PERMISSION_GRANTED }) PhoneGps.start(this)
    }

    private var urlIdx = 0
    private fun loadSandbox(reset: Boolean = false) { if (reset) urlIdx = 0; loadFailed = false; web.loadUrl(SANDBOX_URLS[urlIdx]) }

    private fun showOffline(why: String) {
        val html = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<body style="margin:0;height:100vh;display:flex;align-items:center;justify-content:center;background:#0d0f12;color:#e8eaed;font:15px -apple-system,Roboto,sans-serif">
<div style="text-align:center;max-width:440px;padding:24px"><div style="font-size:20px;font-weight:600;margin-bottom:8px">Can't reach cradle</div>
<div style="color:#9aa0a6;margin-bottom:22px">The sandbox loads live over the tailnet. Check Tailscale is connected on this phone. (${why.replace("<", "")})</div>
<button onclick="FL.reload()" style="font:inherit;padding:12px 22px;border-radius:14px;border:1px solid #ffffff26;background:#ffffff14;color:#e8eaed;margin:4px">Try again</button>
<button onclick="FL.openLab()" style="font:inherit;padding:12px 22px;border-radius:14px;border:1px solid #ffffff26;background:#ffffff14;color:#e8eaed;margin:4px">Flight Lab panel</button></div></body>"""
        web.loadDataWithBaseURL(null, html, "text/html", "utf-8", null)
    }

    private fun js(code: String) { ui.post { if (!loadFailed) web.evaluateJavascript(code, null) } }

    private fun toast(m: String) { Toast.makeText(this, m, Toast.LENGTH_SHORT).show() }

    private fun immersive() {
        if (Build.VERSION.SDK_INT >= 30) {
            window.setDecorFitsSystemWindows(false)
            window.insetsController?.let {
                it.hide(WindowInsets.Type.systemBars())
                it.systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or View.SYSTEM_UI_FLAG_FULLSCREEN or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION or View.SYSTEM_UI_FLAG_LAYOUT_STABLE)
        }
    }

    // ---- diagnostics: this app's logcat + state to cradle (planner/applogs), so problems can be read without adb
    /** Any failure the drone side reports goes to cradle on its own (at most once a minute), so nobody has to screenshot it. */
    private var lastAutoDiag = 0L
    private val FAILURE = Regex("(?i)(failed|can't|couldn't|aborted|error|timed out|refus|not safe|lost the)")
    private fun autoDiagnostics(m: String) {
        if (!FAILURE.containsMatchIn(m) || m.startsWith("Diagnostics")) return
        val now = SystemClock.elapsedRealtime()
        if (now - lastAutoDiag < 60_000) return
        lastAutoDiag = now
        ui.postDelayed({ sendDiagnostics("auto: $m") }, 1500)   // let the failure land in the log first
    }

    private fun sendDiagnostics(why: String) = Thread {
        try {
            val log = Runtime.getRuntime().exec(arrayOf("logcat", "-d", "-t", "1500", "--pid=${android.os.Process.myPid()}")).inputStream.bufferedReader().readText()
            // The newest flight event log (mission/follow states, takeoff results, SDK errors) is the most useful part.
            val events = try {
                java.io.File(getExternalFilesDir(null), "flightlab").listFiles { f -> f.name.endsWith(".events.csv") }
                    ?.maxByOrNull { it.lastModified() }?.let { f -> "events (${f.name}):\n" + f.readLines().takeLast(150).joinToString("\n") } ?: "no event log"
            } catch (t: Throwable) { "event log unreadable: $t" }
            val body = "why: $why\nversion: ${updater.localName} (${updater.localCode})\nandroid: ${Build.VERSION.SDK_INT} ${Build.MODEL}\nstatus: ${Bridge().status()}\n\n$events\n\nlogcat:\n$log"
            val c = (java.net.URL("https://$HOST:8444/api/applog").openConnection() as java.net.HttpURLConnection).apply {
                requestMethod = "POST"; doOutput = true; connectTimeout = 8000; readTimeout = 15000; setRequestProperty("Content-Type", "text/plain; charset=utf-8")
            }
            c.outputStream.use { it.write(body.toByteArray()) }
            val ok = c.responseCode == 200
            if (why == "manual") ui.post { toast(if (ok) "Diagnostics sent." else "Couldn't send diagnostics (${c.responseCode}).") }
        } catch (e: Exception) { if (why == "manual") ui.post { toast("Couldn't send diagnostics: ${e.message}") } }
    }.start()

    // ---- self-update ---------------------------------------------------------------------------
    private fun checkForUpdate(manual: Boolean) = Thread {
        val r = updater.check()
        val local = updater.localCode
        ui.post {
            when {
                r == null -> if (manual) toast("Couldn't reach the update server.")
                r.code <= local -> if (manual) toast("Up to date (${updater.localName}).")
                else -> {
                    js("window.flUpdate&&flUpdate(${JSONObject().put("available", true).put("name", r.name).put("notes", r.notes)})")
                    AlertDialog.Builder(this)
                        .setTitle("Update available")
                        .setMessage("${updater.localName} → ${r.name}\n\n${r.notes.ifBlank { "New Flight Lab build." }}\n\n${"%.0f".format(r.size / 1e6)} MB over the tailnet." +
                            (if (r.mirror != null) "\n\nFast downloads from the Mac (direct connection; needs the Mac awake). Cradle always works but goes through a relay." else ""))
                        .apply {
                            if (r.mirror != null) {
                                setPositiveButton("Fast (Mac)") { _, _ -> download(r, r.mirror, fallback = r.url) }
                                setNeutralButton("From cradle") { _, _ -> download(r, r.url) }
                            } else setPositiveButton("Update") { _, _ -> download(r, r.url) }
                        }
                        .setNegativeButton("Later", null).show()
                }
            }
        }
    }.start()

    private fun download(r: Updater.Remote, url: String, fallback: String? = null) {
        if (Lab.isReady && Lab.ctl.flying) return toast("Land first, then update.")
        val dlg = AlertDialog.Builder(this).setTitle("Downloading ${r.name}").setMessage("Starting…").setCancelable(false).show()
        Thread {
            var err = updater.downloadAndInstall(r, url) { p -> ui.post { dlg.setMessage("${(p * 100).toInt()}%") } }
            if (err != null && fallback != null) {
                ui.post { dlg.setMessage("Mac unreachable, downloading from cradle…") }
                err = updater.downloadAndInstall(r, fallback) { p -> ui.post { dlg.setMessage("From cradle · ${(p * 100).toInt()}%") } }
            }
            ui.post { dlg.dismiss(); if (err != null) toast(err) }
        }.start()
    }

    // ---- JS bridge: window.FL ------------------------------------------------------------------
    inner class Bridge {
        private fun main(f: () -> Unit) { ui.post { try { f() } catch (e: Exception) { this@AppActivity.toast("$e") } } }
        private fun drone(f: () -> Unit) = main { if (Lab.isReady) f() else this@AppActivity.toast("The DJI SDK is still starting.") }
        private fun n(v: Double): Any = if (v.isNaN() || v.isInfinite()) JSONObject.NULL else v

        @JavascriptInterface fun version(): String = JSONObject().put("code", updater.localCode).put("name", updater.localName).toString()

        @JavascriptInterface fun status(): String {
            val sdk = JSONObject().put("init", sdkInit).put("error", sdkError ?: JSONObject.NULL).put("product", productConnected).put("product_id", productId)
            if (!Lab.isReady) return JSONObject().put("registered", false).put("connected", productConnected).put("state", "IDLE").put("sdk", sdk).toString()
            val c = Lab.ctl; val r = Lab.runner; val p = Lab.pilot
            val pos = r.posRel()
            return JSONObject()
                .put("registered", try { SDKManager.getInstance().isRegistered } catch (_: Throwable) { false })
                .put("connected", Lab.droneConnected || productConnected).put("sdk", sdk)
                .put("flying", c.flying).put("motors", c.motorsOn).put("battery", c.battery).put("sats", c.gpsSats)
                .put("lat", n(c.lat)).put("lon", n(c.lon)).put("alt", n(c.altRel)).put("yaw", n(c.yaw))
                .put("gimbal_pitch", n(c.gimbalPitch)).put("vel_ned", org.json.JSONArray().put(n(c.velN)).put(n(c.velE)).put(n(c.velD)))
                .put("pos_rel", JSONObject().put("e", n(pos.e)).put("n", n(pos.n)).put("up", n(pos.u)))
                .put("state", r.state.name).put("info", r.info).put("hold", r.holdReason)
                .put("mission", r.mission?.name ?: JSONObject.NULL).put("remaining_m", n(if (r.mission != null) r.remainingM() else Double.NaN))
                .put("point", r.ctlLabel()).put("poi", r.activePoi).put("landing_confirm", r.landingNeedsConfirm)
                .put("latched", c.latched).put("latch_reason", c.latchReason).put("vs", c.vsEnabled)
                .put("capturing", c.capturing).put("auto_record", c.autoRecord).put("upload", c.uploadStatus).put("video_packets", c.videoPackets)
                .put("video_view", video.visibility == View.VISIBLE)
                .put("health", org.json.JSONArray(c.healthWarnings())).put("device_status", c.deviceStatus())
                .put("compass", JSONObject().put("state", c.compassState).put("calibrating", c.compassCalibrating).put("error", c.compassError))
                .put("product_type", try { dji.sdk.keyvalue.key.ProductKey.KeyProductType.create().get()?.name ?: "" } catch (_: Throwable) { "" })
                .put("home", try { FlightControllerKey.KeyHomeLocation.create().get()?.let { h -> JSONObject().put("lat", h.latitude).put("lon", h.longitude) } ?: JSONObject.NULL } catch (_: Throwable) { JSONObject.NULL })
                .put("follow", JSONObject().put("on", r.following).put("info", r.followInfo)
                    .put("phone_acc", n(PhoneGps.acc)).put("phone_age_s", n(PhoneGps.ageS().coerceAtMost(9999.0))))
                .put("pilot", JSONObject().put("mode", p.mode.name.lowercase()).put("status", p.status).put("want", p.want).put("rtt_ms", n(p.rttMs)))
                .toString()
        }

        /** Hand the sandbox's mission JSON to the runner. Returns "" or an error. */
        @JavascriptInterface fun load(mission: String): String = if (!Lab.isReady) "The DJI SDK is still starting." else Lab.runner.loadText(mission) ?: ""
        @JavascriptInterface fun start(): Unit = drone { Lab.runner.start() }
        @JavascriptInterface fun pause(): Unit = drone { Lab.runner.pause() }
        @JavascriptInterface fun abort(): Unit = drone { Lab.runner.abort() }
        @JavascriptInterface fun land(): Unit = drone { Lab.runner.land() }
        @JavascriptInterface fun confirmLanding(): Unit = drone { Lab.runner.confirmLanding() }
        @JavascriptInterface fun rearm(): Unit = drone { if (Lab.runner.state == MissionRunner.State.MANUAL) Lab.runner.continueAfterManual() else Lab.ctl.rearm() }
        @JavascriptInterface fun pilotMode(m: String): Unit = main {
            if (!Lab.isReady) { pendingAi = m; return@main }
            val r = Lab.runner.state
            if (r in listOf(MissionRunner.State.FLYING, MissionRunner.State.POLICY, MissionRunner.State.TAKEOFF, MissionRunner.State.ARMING)) this@AppActivity.toast("Change the obstacle mode before starting.")
            else applyAi(m)
        }
        /** Show the drone's live video behind the (transparent) page. */
        @JavascriptInterface fun video(on: Boolean): Unit = main { video.visibility = if (on) View.VISIBLE else View.GONE }
        @JavascriptInterface fun openLab(): Unit = drone { startActivity(Intent(this@AppActivity, FlightLabActivity::class.java)) }
        @JavascriptInterface fun openSdk(): Unit = main { startActivity(Intent(this@AppActivity, dji.sampleV5.aircraft.DJIAircraftMainActivity::class.java)) }
        @JavascriptInterface fun diagnostics(): Unit = main { sendDiagnostics("manual") }
        @JavascriptInterface fun checkUpdate(): Unit = main { checkForUpdate(manual = true) }
        @JavascriptInterface fun reload(): Unit = main { loadSandbox(reset = true) }
        @JavascriptInterface fun toast(m: String): Unit = main { this@AppActivity.toast(m) }
        // ---- Videos tab: recordings stored on this phone
        @JavascriptInterface fun recordings(): String = if (!Lab.isReady) "[]" else Lab.ctl.recordings().toString()
        @JavascriptInterface fun storage(): String = if (!Lab.isReady) "{}" else Lab.ctl.storage().toString()
        @JavascriptInterface fun deleteRecording(stamp: String): String = if (!Lab.isReady) "The app is still starting." else Lab.ctl.deleteRecording(stamp)
        @JavascriptInterface fun uploadRecording(stamp: String): String = if (!Lab.isReady) "The app is still starting." else Lab.ctl.uploadRecording(stamp)
        @JavascriptInterface fun setAutoDelete(on: Boolean): Unit { if (Lab.isReady) Lab.ctl.autoDelete = on }
        @JavascriptInterface fun setAutoRecord(on: Boolean): Unit { if (Lab.isReady) Lab.ctl.autoRecord = on }
        @JavascriptInterface fun capture(on: Boolean): Unit = drone { if (on) Lab.ctl.startCapture(recordSd = true) else Lab.ctl.stopCapture() }
        @JavascriptInterface fun follow(distM: Double, heightM: Double): Unit = drone { Lab.runner.startFollow(distM, heightM) }
        @JavascriptInterface fun stopFollow(): Unit = drone { Lab.runner.stopFollow() }
        @JavascriptInterface fun compassCalibration(start: Boolean): String = if (!Lab.isReady) "The DJI SDK is still starting." else if (start) Lab.ctl.startCompassCalibration() else { Lab.ctl.stopCompassCalibration(); "" }
        @JavascriptInterface fun uploadStatus(): String = if (!Lab.isReady) "" else Lab.ctl.uploadStatus
    }

    companion object {
        const val HOST = "cradle.tailb1b51d.ts.net"
        /** With circleback's feedback overlay first, the planner directly as fallback. */
        val SANDBOX_URLS = listOf("https://$HOST:8446/sandbox.html?app=1", "https://$HOST:8444/sandbox.html?app=1")
        const val REQ_LOC = 41
    }
}
