package dji.sampleV5.aircraft.flightlab

import android.app.Activity
import android.graphics.Color
import android.graphics.Typeface
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.Gravity
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager

/** Live video + telemetry + virtual-stick manoeuvres + CSV logging, with RC override. */
class FlightLabActivity : Activity() {

    private lateinit var ctl: FlightLabController
    private lateinit var runner: MissionRunner
    private lateinit var pilot: PilotLink
    private lateinit var pilotBtn: Button
    private lateinit var pauseBtn: Button
    private lateinit var confirmLandBtn: Button
    private lateinit var status: TextView
    private lateinit var telemetry: TextView
    private lateinit var rollBtn: Button
    private lateinit var speedBtn: Button
    private lateinit var logBtn: Button
    private lateinit var captureBtn: Button
    private lateinit var clock: TextView
    private val ui = Handler(Looper.getMainLooper())
    private var surfaceView: SurfaceView? = null
    private val speeds = doubleArrayOf(1.0, 2.0, 3.0, 5.0, 8.0)
    private var speedIdx = 0

    private val streamManager get() = MediaDataCenter.getInstance().cameraStreamManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        Lab.init(this)
        ctl = Lab.ctl; runner = Lab.runner; pilot = Lab.pilot
        setContentView(buildUi())
        ctl.maxHorizontal = speeds[speedIdx]
        ui.post(refresh)
        clock.postOnAnimation(tick)
    }

    private val tick = object : Runnable {
        override fun run() {
            val ms = System.currentTimeMillis() % 100_000
            clock.text = "%02d.%03d".format(ms / 1000, ms % 1000)
            clock.postOnAnimation(this)
        }
    }

    override fun onStart() { super.onStart(); Lab.screenStarted(); Lab.onMessage = { toast(it) } }

    override fun onStop() {
        super.onStop()
        // Never keep flying a script while no Flight Lab screen is visible.
        Lab.screenStopped()
    }

    override fun onDestroy() {
        ui.removeCallbacksAndMessages(null)
        clock.removeCallbacks(tick)
        surfaceView?.holder?.surface?.let { streamManager.removeCameraStreamSurface(it) }
        super.onDestroy()
    }

    // ---- UI ------------------------------------------------------------------
    private fun buildUi(): View {
        val root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }

        surfaceView = SurfaceView(this).apply {
            holder.addCallback(object : SurfaceHolder.Callback {
                override fun surfaceCreated(h: SurfaceHolder) {}
                override fun surfaceChanged(h: SurfaceHolder, format: Int, w: Int, hgt: Int) {
                    streamManager.putCameraStreamSurface(
                        ComponentIndexType.LEFT_OR_MAIN, h.surface, w, hgt, ICameraStreamManager.ScaleType.CENTER_INSIDE
                    )
                }

                override fun surfaceDestroyed(h: SurfaceHolder) {
                    streamManager.removeCameraStreamSurface(h.surface)
                }
            })
        }
        root.addView(surfaceView, FrameLayout.LayoutParams(-1, -1))

        status = TextView(this).apply {
            textSize = 18f
            setTypeface(Typeface.DEFAULT_BOLD)
            setPadding(24, 12, 24, 12)
            setTextColor(Color.WHITE)
        }
        root.addView(status, FrameLayout.LayoutParams(-1, -2, Gravity.TOP))

        telemetry = TextView(this).apply {
            typeface = Typeface.MONOSPACE
            textSize = 12f
            setTextColor(Color.WHITE)
            setBackgroundColor(0x99000000.toInt())
            setPadding(20, 16, 20, 16)
        }
        root.addView(telemetry, FrameLayout.LayoutParams(-2, -2, Gravity.START or Gravity.BOTTOM).apply { setMargins(16, 0, 0, 16) })

        clock = TextView(this).apply {
            typeface = Typeface.MONOSPACE
            textSize = 34f
            setTextColor(Color.YELLOW)
            setBackgroundColor(0xCC000000.toInt())
            setPadding(16, 4, 16, 4)
        }
        root.addView(clock, FrameLayout.LayoutParams(-2, -2, Gravity.TOP or Gravity.START).apply { topMargin = dp(52); leftMargin = dp(12) })

        val panel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(12, 12, 12, 12)
            setBackgroundColor(0x99000000.toInt())
        }
        fun btn(text: String, color: Int = 0xFF37474F.toInt(), onClick: () -> Unit) = Button(this).apply {
            this.text = text
            isAllCaps = false
            setTextColor(Color.WHITE)
            setBackgroundColor(color)
            setOnClickListener { onClick() }
            panel.addView(this, LinearLayout.LayoutParams(-1, -2).apply { setMargins(0, 6, 0, 6) })
        }
        fun header(t: String) = TextView(this).apply { text = t; setTextColor(0xFFB0BEC5.toInt()); textSize = 11f; setPadding(4, 10, 4, 0); panel.addView(this) }
        header("MISSION")
        btn("Load latest mission", 0xFF1565C0.toInt()) { runner.loadLatest() }
        btn("▶ Start mission", 0xFF2E7D32.toInt()) { runner.start() }
        pauseBtn = btn("Pause / Resume") { runner.pause() }
        btn("■ Abort mission (hover)", 0xFFC62828.toInt()) { runner.abort() }
        btn("Land here") { runner.land() }
        confirmLandBtn = btn("Confirm landing", 0xFFEF6C00.toInt()) { runner.confirmLanding() }
        pilotBtn = btn("") {
            if (runner.state in listOf(MissionRunner.State.FLYING, MissionRunner.State.POLICY, MissionRunner.State.TAKEOFF, MissionRunner.State.ARMING)) return@btn toast("Change the obstacle mode before starting.")
            pilot.mode = PilotLink.Mode.values()[(pilot.mode.ordinal + 1) % PilotLink.Mode.values().size]
            labels()
        }
        header("CAPTURE")
        captureBtn = btn("", 0xFFB71C1C.toInt()) {
            if (ctl.capturing) ctl.stopCapture() else ctl.startCapture(recordSd = true)
            labels()
        }
        btn("Upload last capture") { ctl.uploadLastSession(java.text.SimpleDateFormat("yyyy-MM-dd", java.util.Locale.US).format(java.util.Date())) }
        btn("Open planner") {
            startActivity(android.content.Intent(android.content.Intent.ACTION_VIEW, android.net.Uri.parse("https://cradle.tailb1b51d.ts.net:8444/#sim")))
        }
        header("CONTROL / TESTS")
        btn("Enable VS", 0xFF2E7D32.toInt()) { ctl.enableVirtualStick() }
        btn("Disable VS", 0xFFC62828.toInt()) { ctl.disableVirtualStick() }
        btn("Re-arm", 0xFFEF6C00.toInt()) { if (runner.state == MissionRunner.State.MANUAL) runner.continueAfterManual() else ctl.rearm() }
        btn("STOP script (hover)", 0xFFC62828.toInt()) { ctl.abortManeuver("stop pressed"); toast("Stopped. Hovering.") }
        speedBtn = btn("") { speedIdx = (speedIdx + 1) % speeds.size; ctl.maxHorizontal = speeds[speedIdx]; labels() }
        rollBtn = btn("") { ctl.rollIsForward = !ctl.rollIsForward; ctl.event("axis_map rollIsForward=${ctl.rollIsForward}"); labels() }
        btn("AXIS CHECK (0.5 m/s fwd)") { ctl.runManeuver(FlightLabController.axisCheck()) }
        btn("Hover 10 s (wind drift)") { ctl.runManeuver(FlightLabController.hover()) }
        btn("Forward step") { ctl.runManeuver(FlightLabController.forward(speeds[speedIdx])) }
        btn("Lateral step") { ctl.runManeuver(FlightLabController.lateral(speeds[speedIdx])) }
        btn("Reversal +v to -v") { ctl.runManeuver(FlightLabController.reversal(speeds[speedIdx])) }
        btn("Diagonal 2 m/s") { ctl.runManeuver(FlightLabController.diagonal(2.0)) }
        btn("Yaw 60 while fwd 3 m/s") { ctl.runManeuver(FlightLabController.yawWhileMoving()) }
        btn("Vertical +/-1 m/s") { ctl.runManeuver(FlightLabController.vertical()) }
        btn("Yaw 30/60/90 deg/s") { ctl.runManeuver(FlightLabController.yaw()) }
        logBtn = btn("") { if (ctl.logFile == null) ctl.startLogging() else ctl.stopLogging(); labels() }

        val scroll = ScrollView(this).apply { addView(panel) }
        root.addView(scroll, FrameLayout.LayoutParams(dp(220), -1, Gravity.END).apply { topMargin = dp(48) })
        labels()
        return root
    }

    private fun labels() {
        speedBtn.text = "Step speed: ${speeds[speedIdx].toInt()} m/s (tap)"
        rollBtn.text = if (ctl.rollIsForward) "Axis map: roll=fwd (tap)" else "Axis map: pitch=fwd (tap)"
        logBtn.text = if (ctl.logFile == null) "Start log" else "Stop log"
        pilotBtn.text = when (pilot.mode) {
            PilotLink.Mode.OFF -> "Obstacle AI: off (tap)"
            PilotLink.Mode.SHADOW -> "Obstacle AI: shadow, beep only (tap)"
            PilotLink.Mode.POLICY -> "Obstacle AI: LIVE (tap)"
        }
        captureBtn.text = if (ctl.capturing) "■ Stop capture" else "● Capture (4K SD + stream + GPS)"
    }

    private val refresh = object : Runnable {
        override fun run() {
            render()
            ui.postDelayed(this, 200)
        }
    }

    private fun f(v: Double, d: Int = 2) = if (v.isNaN()) "--" else "%.${d}f".format(v)

    private fun render() {
        val c = ctl
        val (st, color) = when {
            c.capturing && !c.latched -> "● CAPTURING ${(android.os.SystemClock.elapsedRealtimeNanos() - c.captureStartNs) / 1_000_000_000}s · stream ${"%.1f".format(c.streamBytes / 1e6)} MB${if (c.sdRecording) " · SD recording" else ""}" to 0xFFB71C1C.toInt()
            runner.state == MissionRunner.State.POLICY && !c.latched -> "OBSTACLE: learned policy flying · ${runner.flyingBy()}  (RC stick = take over)" to 0xFF6A1B9A.toInt()
            runner.state == MissionRunner.State.FLYING && !c.latched && pilot.want != "mission" && pilot.mode == PilotLink.Mode.SHADOW -> "SHADOW: policy would take over now (${pilot.lastCmd?.reason}) · mission continues" to 0xFF8E24AA.toInt()
            runner.state == MissionRunner.State.FLYING && !c.latched -> "MISSION ${runner.mission?.name?.take(28)} · ${runner.ctlLabel()} · ${"%.0f".format(runner.remainingM())} m left  (RC stick = take over)" to 0xFF1565C0.toInt()
            runner.state == MissionRunner.State.MANUAL -> "YOU ARE FLYING · mission waiting. Centre sticks, tap Re-arm & continue" to 0xFFC62828.toInt()
            runner.state == MissionRunner.State.HOLD -> "MISSION HOLD: ${runner.holdReason}" to 0xFFEF6C00.toInt()
            runner.state == MissionRunner.State.PAUSED -> "MISSION PAUSED · hovering" to 0xFFF9A825.toInt()
            runner.state in listOf(MissionRunner.State.TAKEOFF, MissionRunner.State.ARMING, MissionRunner.State.LANDING, MissionRunner.State.ARRIVED) -> "MISSION ${runner.state}: ${runner.info}" to 0xFF1565C0.toInt()
            c.latched -> "RC OVERRIDE LATCHED: ${c.latchReason}. Centre sticks, tap Re-arm." to 0xFFC62828.toInt()
            c.currentManeuver != null -> "RUNNING ${c.currentManeuver?.name} [${c.currentSegment}]  (move RC stick to abort)" to 0xFF1565C0.toInt()
            c.vsEnabled && c.advancedEnabled -> "APP CONTROL (advanced velocity), hovering. RC stick = override" to 0xFF2E7D32.toInt()
            c.vsEnabled -> "Virtual stick on, waiting for advanced mode..." to 0xFFF9A825.toInt()
            else -> "RC CONTROL (virtual stick off). Override armed." to 0xFF455A64.toInt()
        }
        status.text = st
        confirmLandBtn.visibility = if (runner.landingNeedsConfirm) View.VISIBLE else View.GONE
        pauseBtn.text = if (runner.state == MissionRunner.State.MANUAL) "↻ Re-arm & continue" else if (runner.state == MissionRunner.State.FLYING || runner.state == MissionRunner.State.POLICY) "❚❚ Pause mission" else "▶ Resume mission"
        status.setBackgroundColor(color)

        val (bf, br) = c.bodyVelocity()
        val cmd = c.lastCmd
        telemetry.text = buildString {
            appendLine("vel body  fwd ${f(bf)}  right ${f(br)}  up ${f(-c.velD)} m/s")
            appendLine("cmd       fwd ${f(cmd.fwd)}  right ${f(cmd.right)}  up ${f(cmd.up)}  yaw ${f(cmd.yawRate, 0)} dps  sent=${c.lastCmdSent}")
            appendLine("att       pitch ${f(c.pitch, 1)}  roll ${f(c.roll, 1)}  yaw ${f(c.yaw, 1)}")
            appendLine("alt rel   ${f(c.altRel, 1)} m   ultrasonic ${f(c.ultraH, 2)} (used=${c.ultraUsed ?: "?"})")
            appendLine("gps sats  ${c.gpsSats}   flying=${c.flying}  motors=${c.motorsOn}  batt ${c.battery}%")
            appendLine("rc        L(${c.rcLH},${c.rcLV}) R(${c.rcRH},${c.rcRV})  cmds sent ${c.commandsSent}")
            appendLine("gps       ${if (c.lat.isNaN()) "--" else "%.7f, %.7f".format(c.lat, c.lon)}  alt ${f(c.altGps, 1)} m  gimbal y ${f(c.gimbalYaw, 0)} p ${f(c.gimbalPitch, 0)}")
            if (runner.mission != null) appendLine("mission   ${runner.state}  ${runner.ctlLabel()}  off-path ${f(runner.crossTrack, 1)} m  height err ${f(runner.altError, 1)} m  to next ${f(runner.toNext, 1)} m  poi ${runner.activePoi}")
            if (pilot.mode != PilotLink.Mode.OFF) {
                val pc = pilot.lastCmd
                appendLine("pilot     ${pilot.mode} ${pilot.status}  rtt ${f(pilot.rttMs, 0)} ms  frames ${pilot.framesSent}" +
                    (pc?.let { "  hazard ${it.hazard} p_brake ${f(it.pBrake)} clear ${f(it.clearM, 1)} m want ${it.want} srv ${f(it.serverMs, 0)} ms" } ?: ""))
            }
            if (c.uploadStatus.isNotEmpty()) appendLine("upload    ${c.uploadStatus}")
            appendLine("mode      ${c.flightModeString}  wind ${c.windWarning}  gimbal ${f(c.gimbalPitch, 1)}  video pkts ${c.videoPackets}")
            append("log       ${c.logFile?.name ?: "off"}  rows ${c.logRows}")
        }
    }

    private fun toast(msg: String) = Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()
}
