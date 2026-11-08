package dji.sampleV5.aircraft.flightlab

import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.media.AudioManager
import android.media.ToneGenerator
import android.os.SystemClock
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okio.ByteString
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Pilot link to cradle (docs/pilot-protocol.md in dji-rl): streams 4:3 camera frames plus
 * telemetry at about 10 Hz to the hazard/policy server and keeps its latest decision.
 * It never touches the sticks itself; the MissionRunner decides what to do with `want`/`sticks`.
 */
class PilotLink(private val ctl: FlightLabController, private val runner: MissionRunner) {

    enum class Mode { OFF, SHADOW, POLICY }

    data class Cmd(
        val seq: Long, val hazard: Boolean, val pBrake: Double, val clearM: Double, val want: String,
        val fwd: Double, val right: Double, val up: Double, val yawRate: Double,
        val rejoinIdx: Int?, val reason: String, val serverMs: Double, val rxNs: Long,
    )

    @Volatile var mode = Mode.SHADOW
    @Volatile var connected = false; private set
    @Volatile var ready = false; private set
    @Volatile var lastCmd: Cmd? = null; private set
    @Volatile var rttMs = Double.NaN; private set
    @Volatile var framesSent = 0L; private set
    @Volatile var status = "off"; private set
    @Volatile var want = "mission"; private set
    var onWant: (Cmd) -> Unit = {}

    private var ws: WebSocket? = null
    private var seq = 0L
    private var lastFrameNs = 0L
    private val encoding = AtomicBoolean(false)
    private val enc = Executors.newSingleThreadExecutor { Thread(it, "pilot-enc") }
    private val pinger = Executors.newSingleThreadScheduledExecutor { Thread(it, "pilot-ping") }
    private val http = OkHttpClient.Builder().pingInterval(5, TimeUnit.SECONDS).readTimeout(0, TimeUnit.MILLISECONDS).build()
    private val beeper = try { ToneGenerator(AudioManager.STREAM_ALARM, 100) } catch (_: Exception) { null }
    private val out = ByteArrayOutputStream(32 * 1024)
    private val small = ByteArray(W * H * 3 / 2)

    private val frameListener = ICameraStreamManager.CameraFrameListener { data, offset, length, width, height, format ->
        val now = SystemClock.elapsedRealtimeNanos()
        if (!ready || now - lastFrameNs < FRAME_PERIOD_NS || format != ICameraStreamManager.FrameFormat.NV21) return@CameraFrameListener
        if (length < width * height * 3 / 2 || !encoding.compareAndSet(false, true)) return@CameraFrameListener  // drop if busy
        lastFrameNs = now
        // Downscale now (cheap), JPEG-encode off the decoder thread.
        cropScaleNv21(data, offset, width, height)
        val header = header(now)
        enc.execute {
            try {
                out.reset()
                YuvImage(small, ImageFormat.NV21, W, H, null).compressToJpeg(Rect(0, 0, W, H), JPEG_Q, out)
                val h = header.toString().toByteArray()
                val buf = ByteBuffer.allocate(4 + h.size + out.size()).putInt(h.size).put(h).put(out.toByteArray())
                if (ws?.send(ByteString.of(*buf.array())) == true) framesSent++
            } catch (t: Throwable) { ctl.event("pilot frame error $t") } finally { encoding.set(false) }
        }
    }

    fun connect(missionText: String) {
        if (mode == Mode.OFF) return
        close()
        status = "connecting…"
        ws = http.newWebSocket(Request.Builder().url(URL).build(), object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                connected = true; status = "connected, waiting for ready"
                val hello = JSONObject().put("type", "hello").put("mission", JSONObject(missionText)).put("mode", if (mode == Mode.POLICY) "policy" else "shadow")
                webSocket.send(hello.toString())
                ctl.event("pilot_open mode=$mode")
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                val j = try { JSONObject(text) } catch (_: Exception) { return }
                when (j.optString("type")) {
                    "ready" -> {
                        ready = true
                        val warn = j.optJSONArray("warnings")?.let { if (it.length() > 0) " warnings: $it" else "" } ?: ""
                        status = "ready (${j.optString("scene")}, ${j.optString("mode")})$warn"
                        ctl.event("pilot_ready $text")
                    }
                    "error" -> { status = "server error: ${j.optString("error")}"; ctl.event("pilot_error $text") }
                    "pong" -> rttMs = (SystemClock.elapsedRealtime() - j.optLong("t_ms")).toDouble()
                    "cmd" -> handleCmd(j)
                }
            }

            override fun onMessage(webSocket: WebSocket, bytes: ByteString) {}
            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) = dropped("closed: $reason")
            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) = dropped("failed: ${t.message}")
        })
        MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(ComponentIndexType.LEFT_OR_MAIN, ICameraStreamManager.FrameFormat.NV21, frameListener)
        pinger.scheduleAtFixedRate({ if (connected) ws?.send(JSONObject().put("type", "ping").put("t_ms", SystemClock.elapsedRealtime()).toString()) }, 1, 2, TimeUnit.SECONDS)
    }

    fun close() {
        MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener)
        ws?.close(1000, "bye"); ws = null
        connected = false; ready = false; want = "mission"; status = if (mode == Mode.OFF) "off" else "disconnected"
    }

    fun shutdown() { close(); pinger.shutdownNow(); enc.shutdownNow(); beeper?.release() }

    /** The newest command, or null when it's older than [maxAgeMs] (the policy then hovers). */
    fun freshCmd(maxAgeMs: Long = 300): Cmd? = lastCmd?.takeIf { connected && (SystemClock.elapsedRealtimeNanos() - it.rxNs) / 1_000_000 <= maxAgeMs }

    private fun handleCmd(j: JSONObject) {
        val s = j.optJSONObject("sticks")
        val r = j.optJSONObject("rejoin")
        val c = Cmd(
            seq = j.optLong("seq"), hazard = j.optBoolean("hazard"), pBrake = j.optDouble("p_brake", 0.0), clearM = j.optDouble("clear_m", Double.NaN),
            want = j.optString("want", "mission"),
            fwd = s?.optDouble("fwd_mps", 0.0) ?: 0.0, right = s?.optDouble("right_mps", 0.0) ?: 0.0,
            up = s?.optDouble("up_mps", 0.0) ?: 0.0, yawRate = s?.optDouble("yaw_rate_dps", 0.0) ?: 0.0,
            rejoinIdx = r?.optInt("route_idx"), reason = j.optString("reason", ""), serverMs = j.optDouble("server_ms", Double.NaN),
            rxNs = SystemClock.elapsedRealtimeNanos(),
        )
        lastCmd = c
        if (c.want != want) {
            ctl.event("pilot_want ${want} -> ${c.want} reason=${c.reason} p_brake=${c.pBrake} rejoin=${c.rejoinIdx}")
            if (c.want == "policy" || c.want == "hover") beep()
            want = c.want
            runner.onPilotWant(c)
            onWant(c)
        }
    }

    private fun dropped(why: String) {
        if (!connected && ws == null) return
        connected = false; ready = false; status = "link lost ($why)"
        ctl.event("pilot_drop $why")
        if (want != "mission") { want = "mission"; beep() }
        runner.onPilotLost(why)
    }

    private fun beep() = try { beeper?.startTone(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 400) } catch (_: Exception) { null }

    private fun header(now: Long): JSONObject {
        val p = runner.posRel()
        val c = ctl.lastCmd
        val ground = if (ctl.ultraUsed == true && (now - ctl.ultraT) < 1_000_000_000L) ctl.ultraH else Double.NaN
        fun n(v: Double): Any = if (v.isNaN()) JSONObject.NULL else v
        return JSONObject()
            .put("seq", ++seq).put("t_ms", now / 1_000_000)
            .put("pos_rel", JSONObject().put("e", n(p.e)).put("n", n(p.n)).put("up", n(p.u)))
            .put("vel_ned", JSONArray().put(n(ctl.velN)).put(n(ctl.velE)).put(n(ctl.velD)))
            .put("yaw_deg", n(ctl.yaw)).put("gimbal_pitch_deg", n(ctl.gimbalPitch))
            .put("flying_by", runner.flyingBy())
            // Velocity actually commanded (our virtual stick is velocity mode, not normalised sticks).
            .put("cmd_vel", JSONObject().put("fwd_mps", c.fwd).put("right_mps", c.right).put("up_mps", c.up).put("yaw_rate_dps", c.yawRate))
            .put("ground_h_m", if (ground.isNaN()) JSONObject.NULL else ground)
    }

    /** Centre-crop to 4:3 and nearest-neighbour scale to W x H, NV21 in and out. */
    private fun cropScaleNv21(src: ByteArray, off: Int, w: Int, h: Int) {
        var cw = w; var ch = w * 3 / 4
        if (ch > h) { ch = h; cw = h * 4 / 3 }
        val x0 = ((w - cw) / 2) and 1.inv(); val y0 = ((h - ch) / 2) and 1.inv()
        for (j in 0 until H) {
            val sy = y0 + j * ch / H
            val row = off + sy * w
            for (i in 0 until W) small[j * W + i] = src[row + x0 + i * cw / W]
        }
        val uv = off + w * h
        for (j in 0 until H / 2) {
            val sy = (y0 + 2 * j * ch / H) / 2
            for (i in 0 until W / 2) {
                val sx = (x0 + 2 * i * cw / W) / 2
                val s = uv + sy * w + 2 * sx
                val d = W * H + j * W + 2 * i
                small[d] = src[s]; small[d + 1] = src[s + 1]
            }
        }
    }

    companion object {
        const val URL = "wss://cradle.tailb1b51d.ts.net:8445/"
        const val W = 320
        const val H = 240
        const val JPEG_Q = 80
        const val FRAME_PERIOD_NS = 95_000_000L   // about 10 Hz
    }
}
