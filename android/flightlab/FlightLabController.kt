package dji.sampleV5.aircraft.flightlab

import android.content.Context
import android.os.SystemClock
import android.util.Log
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.create
import dji.v5.et.action
import dji.v5.et.listen
import dji.v5.et.get
import dji.v5.et.set
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.value.camera.CameraMode
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.datacenter.camera.StreamInfo
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener
import java.io.BufferedWriter
import java.io.BufferedOutputStream
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Flight Lab: advanced-mode virtual stick (body-frame velocity, vertical velocity,
 * yaw rate) sent at 25 Hz, scripted manoeuvres, a 50 Hz CSV telemetry logger, and
 * an RC override latch.
 *
 * Priority: physical RC > this app. While virtual stick is enabled, any RC stick
 * input beyond the deadband (or virtual stick being dropped by anything else, e.g.
 * the RC pause button) latches: the manoeuvre aborts, commands stop, advanced mode
 * and virtual stick are disabled, and nothing is sent until the pilot centres the
 * sticks and taps Re-arm.
 */
class FlightLabController(private val context: Context) {

    // ---- setpoint: body frame, SI units --------------------------------------
    data class Setpoint(val fwd: Double = 0.0, val right: Double = 0.0, val up: Double = 0.0, val yawRate: Double = 0.0)

    data class Segment(val label: String, val seconds: Double, val sp: Setpoint)

    data class Maneuver(val name: String, val segments: List<Segment>) {
        val totalSeconds get() = segments.sumOf { it.seconds }
    }

    // ---- telemetry (latest value + phone receive time) -----------------------
    @Volatile var velN = Double.NaN; @Volatile var velE = Double.NaN; @Volatile var velD = Double.NaN
    @Volatile var velT = 0L
    @Volatile var pitch = Double.NaN; @Volatile var roll = Double.NaN; @Volatile var yaw = Double.NaN
    @Volatile var attT = 0L
    @Volatile var lat = Double.NaN; @Volatile var lon = Double.NaN; @Volatile var altGps = Double.NaN
    @Volatile var locT = 0L
    @Volatile var altRel = Double.NaN; @Volatile var altT = 0L
    @Volatile var ultraH = Double.NaN; @Volatile var ultraT = 0L
    @Volatile var ultraUsed: Boolean? = null
    @Volatile var gpsSats = -1
    @Volatile var flying = false
    @Volatile var motorsOn = false
    @Volatile var battery = -1
    @Volatile var flightModeString = ""
    @Volatile var fcFlightMode = ""
    @Volatile var windWarning = ""
    @Volatile var gimbalPitch = Double.NaN; @Volatile var gimbalYaw = Double.NaN; @Volatile var gimbalRoll = Double.NaN; @Volatile var gimbalT = 0L
    @Volatile var videoPackets = 0L
    @Volatile var rcLH = 0; @Volatile var rcLV = 0; @Volatile var rcRH = 0; @Volatile var rcRV = 0

    // ---- control state --------------------------------------------------------
    @Volatile var vsEnabled = false; private set
    @Volatile var advancedEnabled = false; private set
    @Volatile var latched = false; private set
    @Volatile var latchReason = ""; private set
    @Volatile var lastAuthorityReason = ""; private set
    @Volatile private var selfDisabling = false

    /** DJI's body-frame velocity mode axis naming is notoriously confusing; this is switchable after an AXIS CHECK. */
    @Volatile var rollIsForward = true

    @Volatile var maxHorizontal = 3.0
    @Volatile var currentManeuver: Maneuver? = null; private set
    @Volatile private var maneuverStartNs = 0L
    @Volatile var lastCmd = Setpoint(); private set
    @Volatile var lastCmdSent = false; private set
    @Volatile var currentSegment = ""; private set
    @Volatile var commandsSent = 0L; private set
    /** Mission runner hook: called at 25 Hz when no manoeuvre runs; returns the next setpoint. */
    @Volatile var autopilot: (() -> Setpoint)? = null
    @Volatile var autopilotLabel = ""
    /** Called (on the control thread) when the RC latch trips while the autopilot is flying. */
    @Volatile var onAutopilotKilled: (String) -> Unit = {}

    // ---- logging --------------------------------------------------------------
    @Volatile var logFile: File? = null; private set
    @Volatile var logRows = 0L; private set
    private var logWriter: BufferedWriter? = null
    private var eventWriter: BufferedWriter? = null
    private var cmdWriter: BufferedWriter? = null
    private var frameWriter: BufferedWriter? = null
    private val logLock = Any()

    var onMessage: (String) -> Unit = {}

    private val exec = Executors.newSingleThreadScheduledExecutor { Thread(it, "flightlab").apply { priority = Thread.MAX_PRIORITY } }
    private var cmdTask: ScheduledFuture<*>? = null
    private var logTask: ScheduledFuture<*>? = null

    private val vsm get() = VirtualStickManager.getInstance()

    private val streamListener = ICameraStreamManager.ReceiveStreamListener { data, offset, length, info: StreamInfo ->
        val n = now()
        videoPackets++
        if (capturing) synchronized(logLock) {
            val out = streamOut ?: run {
                // Raw Annex-B elementary stream; pts/arrival per packet are in video.csv for muxing later.
                val ext = if (info.mimeType == ICameraStreamManager.MimeType.H265) "h265" else "h264"
                val f = File(logDir(), "flightlab_${logStamp}.stream.$ext")
                event("stream_record_start ${f.name} ${info.width}x${info.height}@${info.frameRate}")
                sessionFiles += f
                BufferedOutputStream(f.outputStream(), 1 shl 20).also { streamOut = it }
            }
            out.write(data, offset, length)
            streamBytes += length
        }
        synchronized(logLock) {
            frameWriter?.apply {
                write("$n,${System.currentTimeMillis()},${info.presentationTimeMs},${if (info.isKeyFrame) 1 else 0},$length,${info.width},${info.height},${info.frameRate}")
                newLine()
            }
        }
    }

    fun start() {
        listenTelemetry()
        vsm.setVirtualStickStateListener(object : VirtualStickStateListener {
            override fun onVirtualStickStateUpdate(state: VirtualStickState) {
                val was = vsEnabled
                vsEnabled = state.isVirtualStickEnable
                advancedEnabled = state.isVirtualStickAdvancedModeEnabled
                if (was != vsEnabled) event("vs_state enabled=$vsEnabled advanced=$advancedEnabled owner=${state.currentFlightControlAuthorityOwner}")
                if (was && !vsEnabled && !selfDisabling && !latched) {
                    trip("virtual stick dropped externally (owner=${state.currentFlightControlAuthorityOwner}, reason=$lastAuthorityReason)")
                }
            }

            override fun onChangeReasonUpdate(reason: FlightControlAuthorityChangeReason) {
                lastAuthorityReason = reason.name
                event("authority_change $reason")
            }
        })
        MediaDataCenter.getInstance().cameraStreamManager.addReceiveStreamListener(ComponentIndexType.LEFT_OR_MAIN, streamListener)
        cmdTask = exec.scheduleAtFixedRate({ safe { commandTick() } }, 0, CMD_PERIOD_MS, TimeUnit.MILLISECONDS)
        logTask = exec.scheduleAtFixedRate({ safe { logTick() } }, 0, LOG_PERIOD_MS, TimeUnit.MILLISECONDS)
        exec.scheduleAtFixedRate({ safe { storageGuard() } }, 5, 5, TimeUnit.SECONDS)
        exec.scheduleAtFixedRate({ safe { pollTelemetry() } }, 500, 200, TimeUnit.MILLISECONDS)
    }

    /** Leaving the screen hands control back to the RC. */
    fun stop() {
        abortManeuver("screen closed")
        stopCapture()
        if (vsEnabled) disableVirtualStick()
        cmdTask?.cancel(false)
        logTask?.cancel(false)
        exec.execute { stopLogging() }
        exec.shutdown()
        MediaDataCenter.getInstance().cameraStreamManager.removeReceiveStreamListener(streamListener)
        KeyManager.getInstance().cancelListen(this)
        vsm.clearAllVirtualStickStateListener()
    }

    // ---- telemetry -----------------------------------------------------------
    private fun now() = SystemClock.elapsedRealtimeNanos()

    private fun num(v: Any?): Double = (v as? Number)?.toDouble() ?: Double.NaN

    /**
     * listen() only fires when a value changes, so a drone sitting still on the ground (altitude 0,
     * same GPS fix) looks stale to the freshness checks. While the aircraft is connected, re-read the
     * current values at 5 Hz so "fresh" means "the SDK has a current value", not "it moved".
     */
    private fun pollTelemetry() {
        // Only an explicit "disconnected" stops the poll: KeyConnection.get() can read null even with the drone up.
        if (linkDown) return
        FlightControllerKey.KeyAltitude.create().get()?.let { altRel = num(it); altT = now() }
        FlightControllerKey.KeyAircraftLocation3D.create().get()?.let { l ->
            val la = num(l.latitude); val lo = num(l.longitude)
            if (!la.isNaN() && !lo.isNaN() && !(la == 0.0 && lo == 0.0)) { lat = la; lon = lo; altGps = num(l.altitude); locT = now() }
        }
        FlightControllerKey.KeyGPSSatelliteCount.create().get()?.let { gpsSats = num(it).let { d -> if (d.isNaN()) -1 else d.toInt() } }
        BatteryKey.KeyChargeRemainingInPercent.create().get()?.let { battery = num(it).let { d -> if (d.isNaN()) -1 else d.toInt() } }
    }

    @Volatile private var linkDown = false

    private fun listenTelemetry() {
        FlightControllerKey.KeyConnection.create().listen(this) { c ->
            linkDown = c == false
            if (linkDown) { locT = 0L; altT = 0L }
        }
        FlightControllerKey.KeyAircraftVelocity.create().listen(this) {
            it?.let { v -> velN = num(v.x); velE = num(v.y); velD = num(v.z); velT = now() }
        }
        FlightControllerKey.KeyAircraftAttitude.create().listen(this) {
            it?.let { a -> pitch = num(a.pitch); roll = num(a.roll); yaw = num(a.yaw); attT = now() }
        }
        FlightControllerKey.KeyAircraftLocation3D.create().listen(this) {
            it?.let { l -> lat = num(l.latitude); lon = num(l.longitude); altGps = num(l.altitude); locT = now() }
        }
        FlightControllerKey.KeyAltitude.create().listen(this) { altRel = num(it); altT = now() }
        FlightControllerKey.KeyUltrasonicHeight.create().listen(this) { ultraH = num(it); ultraT = now() }
        FlightControllerKey.KeyIsUltrasonicUsed.create().listen(this) { ultraUsed = it }
        FlightControllerKey.KeyGPSSatelliteCount.create().listen(this) { gpsSats = num(it).let { d -> if (d.isNaN()) -1 else d.toInt() } }
        FlightControllerKey.KeyIsFlying.create().listen(this) { v ->
            val f = v == true
            if (f != flying) event("flying=$f")
            flying = f
        }
        FlightControllerKey.KeyAreMotorsOn.create().listen(this) { v ->
            val m = v == true
            if (m != motorsOn) event("motors_on=$m")
            motorsOn = m
        }
        FlightControllerKey.KeyFlightModeString.create().listen(this) { v ->
            val m = v?.toString() ?: ""
            if (m != flightModeString) event("flight_mode $m")
            flightModeString = m
        }
        FlightControllerKey.KeyFCFlightMode.create().listen(this) { fcFlightMode = it?.toString() ?: "" }
        FlightControllerKey.KeyCompassCalibrationStatus.create().listen(this) { v ->
            val st = v?.name ?: ""
            if (st != compassState) event("compass_cal $st")
            compassState = st
            if (st == "SUCCEEDED") onMessage("Compass calibrated. Power-cycle the drone if it still warns.")
            if (st == "FAILED") onMessage("Compass calibration failed: move away from metal and try again.")
        }
        FlightControllerKey.KeyIsCompassCalibrating.create().listen(this) { compassCalibrating = it == true }
        FlightControllerKey.KeyCompassHasError.create().listen(this) { compassError = it == true }
        FlightControllerKey.KeyWindWarning.create().listen(this) { v ->
            val w = v?.toString() ?: ""
            if (w != windWarning) event("wind_warning $w")
            windWarning = w
        }
        GimbalKey.KeyGimbalAttitude.create().listen(this) { it?.let { a -> gimbalPitch = num(a.pitch); gimbalYaw = num(a.yaw); gimbalRoll = num(a.roll); gimbalT = now() } }
        BatteryKey.KeyChargeRemainingInPercent.create().listen(this) { battery = num(it).let { d -> if (d.isNaN()) -1 else d.toInt() } }
        RemoteControllerKey.KeyStickLeftHorizontal.create().listen(this) { rcLH = it ?: 0; checkRc() }
        RemoteControllerKey.KeyStickLeftVertical.create().listen(this) { rcLV = it ?: 0; checkRc() }
        RemoteControllerKey.KeyStickRightHorizontal.create().listen(this) { rcRH = it ?: 0; checkRc() }
        RemoteControllerKey.KeyStickRightVertical.create().listen(this) { rcRV = it ?: 0; checkRc() }
    }

    // ---- compass calibration (same flow as DJI Fly: level spin, then nose-down spin) ----
    @Volatile var compassState = ""; private set
    @Volatile var compassError = false; private set
    @Volatile var compassCalibrating = false; private set

    fun startCompassCalibration(): String {
        if (flying || motorsOn) return "Land and stop the motors first."
        FlightControllerKey.KeyStartCompassCalibration.create().action({ event("compass_cal start ok") },
            { e: IDJIError -> event("compass_cal start FAILED $e"); onMessage("Compass calibration couldn't start: $e") })
        return ""
    }

    fun stopCompassCalibration() {
        FlightControllerKey.KeyStopCompassCalibration.create().action({ event("compass_cal stopped") }, { e: IDJIError -> event("compass_cal stop FAILED $e") })
    }

    /** What DJI Fly would warn about: the drone's own health messages (compass, IMU, no-fly zone, battery...). */
    fun healthWarnings(): List<String> = try {
        dji.v5.manager.diagnostic.DeviceHealthManager.getInstance().currentDJIDeviceHealthInfos
            .map { h -> "${h.title()}${if (h.description().isNullOrBlank() || h.description() == h.title()) "" else ": " + h.description()} [${h.warningLevel()}]" }
    } catch (t: Throwable) { listOf("health unavailable: $t") }

    fun deviceStatus(): String = try {
        dji.v5.manager.diagnostic.DeviceStatusManager.getInstance().currentDJIDeviceStatus.let { "${it} (${it.description()})" }
    } catch (t: Throwable) { "" }

    fun sticksCentered() = maxOf(abs(rcLH), abs(rcLV), abs(rcRH), abs(rcRV)) <= RC_DEADBAND

    /** Velocity in the body frame (forward, right), from NED velocity and yaw. */
    fun bodyVelocity(): Pair<Double, Double> {
        val y = Math.toRadians(yaw)
        val fwd = velN * cos(y) + velE * sin(y)
        val right = -velN * sin(y) + velE * cos(y)
        return fwd to right
    }

    // ---- override latch ------------------------------------------------------
    private fun checkRc() {
        if (!latched && vsEnabled && !sticksCentered()) trip("RC stick input lh=$rcLH lv=$rcLV rh=$rcRH rv=$rcRV")
    }

    private fun trip(reason: String) {
        synchronized(this) {
            if (latched) return
            latched = true
            latchReason = reason
        }
        Log.w(TAG, "RC OVERRIDE LATCHED: $reason")
        event("LATCH $reason")
        currentManeuver = null
        if (autopilot != null) { autopilot = null; onAutopilotKilled(reason) }
        lastCmd = Setpoint()
        selfDisabling = true
        vsm.setVirtualStickAdvancedModeEnabled(false)
        vsm.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { selfDisabling = false; event("latch: virtual stick disabled, RC has control") }
            override fun onFailure(error: IDJIError) { selfDisabling = false; event("latch: disableVirtualStick FAILED $error") }
        })
        onMessage("RC OVERRIDE: app locked. Centre sticks, tap Re-arm.")
    }

    fun rearm() {
        when {
            !latched -> onMessage("Not latched.")
            !sticksCentered() -> onMessage("Centre the RC sticks first.")
            else -> {
                latched = false
                latchReason = ""
                event("rearmed by pilot")
                onMessage("Re-armed. Tap Enable to hand control to the app.")
            }
        }
    }

    // ---- virtual stick -------------------------------------------------------
    fun enableVirtualStick() {
        if (latched) return onMessage("Latched: centre sticks and Re-arm first.")
        if (!sticksCentered()) return onMessage("Centre the RC sticks first.")
        lastCmd = Setpoint()
        vsm.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                vsm.setVirtualStickAdvancedModeEnabled(true)
                event("enableVirtualStick ok, advanced mode requested")
                onMessage("Virtual stick ON (advanced, velocity). Hovering.")
            }

            override fun onFailure(error: IDJIError) {
                event("enableVirtualStick FAILED $error")
                onMessage("Enable failed: $error")
            }
        })
    }

    fun disableVirtualStick() {
        abortManeuver("disable pressed")
        selfDisabling = true
        vsm.setVirtualStickAdvancedModeEnabled(false)
        vsm.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { selfDisabling = false; event("disableVirtualStick ok"); onMessage("Virtual stick OFF. RC has control.") }
            override fun onFailure(error: IDJIError) { selfDisabling = false; event("disableVirtualStick FAILED $error"); onMessage("Disable failed: $error") }
        })
    }

    // ---- manoeuvres ----------------------------------------------------------
    fun runManeuver(m: Maneuver) {
        when {
            latched -> return onMessage("Latched. Re-arm first.")
            !vsEnabled || !advancedEnabled -> return onMessage("Enable virtual stick first.")
            !flying -> return onMessage("Take off and hover first.")
            currentManeuver != null -> return onMessage("A manoeuvre is already running.")
            autopilot != null -> return onMessage("A mission is flying. Abort it first.")
        }
        if (logFile == null) startLogging()
        maneuverStartNs = now()
        currentManeuver = m
        event("maneuver_start ${m.name} (${"%.1f".format(m.totalSeconds)} s)")
        onMessage("Running ${m.name}. Move an RC stick to abort.")
    }

    fun abortManeuver(why: String) {
        val m = currentManeuver ?: return
        currentManeuver = null
        lastCmd = Setpoint()
        event("maneuver_abort ${m.name}: $why")
    }

    private fun currentSetpoint(): Setpoint {
        val m = currentManeuver ?: run {
            val ap = autopilot ?: return Setpoint().also { currentSegment = "" }
            return ap().also { currentSegment = autopilotLabel }
        }
        var t = (now() - maneuverStartNs) / 1e9
        for (s in m.segments) {
            if (t < s.seconds) {
                currentSegment = s.label
                return s.sp
            }
            t -= s.seconds
        }
        currentManeuver = null
        currentSegment = ""
        event("maneuver_done ${m.name}")
        onMessage("${m.name} done. Hovering.")
        return Setpoint()
    }

    private fun clamp(sp: Setpoint): Setpoint {
        val h = maxHorizontal
        fun c(v: Double, lim: Double) = v.coerceIn(-lim, lim)
        return Setpoint(c(sp.fwd, h), c(sp.right, h), c(sp.up, MAX_VERTICAL), c(sp.yawRate, MAX_YAW_RATE))
    }

    private fun commandTick() {
        if (latched || !vsEnabled || !advancedEnabled) {
            lastCmdSent = false
            if (latched || !vsEnabled) currentManeuver = null
            if (latched && autopilot != null) { autopilot = null; onAutopilotKilled("latched") }
            return
        }
        val sp = clamp(currentSetpoint())
        val p = VirtualStickFlightControlParam().apply {
            rollPitchControlMode = RollPitchControlMode.VELOCITY
            verticalControlMode = VerticalControlMode.VELOCITY
            yawControlMode = YawControlMode.ANGULAR_VELOCITY
            rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
            if (rollIsForward) { roll = sp.fwd; pitch = sp.right } else { pitch = sp.fwd; roll = sp.right }
            yaw = sp.yawRate
            verticalThrottle = sp.up
        }
        val tSend = now()
        vsm.sendVirtualStickAdvancedParam(p)
        synchronized(logLock) {
            cmdWriter?.apply {
                write("$tSend,${currentManeuver?.name ?: if (autopilot != null) "MISSION" else ""},$currentSegment,${sp.fwd},${sp.right},${sp.up},${sp.yawRate},${p.roll},${p.pitch},${p.yaw},${p.verticalThrottle}")
                newLine()
            }
        }
        lastCmd = sp
        lastCmdSent = true
        commandsSent++
    }

    // ---- capture: live stream to file + 50 Hz GPS/attitude/gimbal log + optional SD 4K recording ----
    @Volatile var capturing = false; private set
    @Volatile var sdRecording = false; private set
    @Volatile var streamBytes = 0L; private set
    @Volatile var captureStartNs = 0L; private set
    @Volatile var uploadStatus = ""; private set
    private var streamOut: BufferedOutputStream? = null
    private var logStamp = ""
    private val sessionFiles = mutableListOf<File>()
    @Volatile var lastSession: List<File> = emptyList(); private set

    private fun logDir() = File(context.getExternalFilesDir(null), "flightlab").apply { mkdirs() }

    fun startCapture(recordSd: Boolean) {
        if (capturing) return
        if (logDir().usableSpace < MIN_FREE_START) { onMessage("Not recording: under 2 GB free on the phone. Delete uploaded videos in Videos."); return }
        if (logFile != null) stopLogging()
        startLogging()
        streamBytes = 0
        captureStartNs = now()
        capturing = true
        event("capture_start recordSd=$recordSd")
        if (recordSd) startSdRecord()
        onMessage(if (recordSd) "Capturing: stream + GPS log + 4K on SD." else "Capturing: stream + GPS log.")
    }

    /** 4K to the drone's SD card. Missions call this once airborne, never alongside the takeoff command. A failure only logs. */
    fun startSdRecord() {
        if (sdRecording || !capturing) return
        CameraKey.KeyCameraMode.create().set(CameraMode.VIDEO_NORMAL)
        CameraKey.KeyStartRecord.create().action({
            sdRecording = true; event("sd_record_start ok")
        }, { e: IDJIError -> event("sd_record_start FAILED $e (SD card full or missing?)") })
    }

    fun stopCapture() {
        if (!capturing) return
        if (sdRecording) CameraKey.KeyStopRecord.create().action({ event("sd_record_stop ok") }, { e: IDJIError -> event("sd_record_stop FAILED $e") })
        sdRecording = false
        capturing = false
        synchronized(logLock) { streamOut?.close(); streamOut = null }
        event("capture_stop stream_bytes=$streamBytes seconds=${(now() - captureStartNs) / 1_000_000_000}")
        stopLogging()
        lastSession = sessionFiles.filter { it.exists() && it.length() > 0 }.toList()
        onMessage("Capture saved: ${lastSession.size} files, ${"%.1f".format(lastSession.sumOf { it.length() } / 1e6)} MB. Tap Upload.")
    }

    /** Streams each file to cradle's tailnet upload server (data/raw/<date>/{drone,telemetry}/). */
    fun uploadLastSession(date: String) {
        val files = lastSession
        if (files.isEmpty()) return onMessage("Nothing captured yet.")
        uploadFiles(files, date)
    }

    // ---- recordings on this phone (the app's Videos tab) ----------------------
    private val prefs get() = context.getSharedPreferences("flightlab", Context.MODE_PRIVATE)
    /** Record every mission/follow flight (phone stream + drone SD 4K). Off by default: Rec records on demand. */
    var autoRecord: Boolean
        get() = prefs.getBoolean("autoRecord", false)
        set(v) { prefs.edit().putBoolean("autoRecord", v).apply() }
    var autoDelete: Boolean
        get() = prefs.getBoolean("autoDelete", true)
        set(v) { prefs.edit().putBoolean("autoDelete", v).apply() }
    private val uploadedFile get() = File(logDir(), ".uploaded")
    private fun uploadedStamps(): Set<String> = try { uploadedFile.readLines().toSet() } catch (_: Throwable) { emptySet() }
    @Volatile private var uploadingStamp: String? = null

    private fun stampOf(f: File) = Regex("flightlab_(\\d{8}_\\d{6})").find(f.name)?.groupValues?.get(1)

    /** Sessions in the app's folder, newest first. Video = the raw drone stream (the big file). */
    fun recordings(): org.json.JSONArray {
        val up = uploadedStamps()
        val bySession = (logDir().listFiles() ?: emptyArray()).filter { it.isFile && stampOf(it) != null }.groupBy { stampOf(it)!! }
        val out = org.json.JSONArray()
        for ((stamp, files) in bySession.toSortedMap(compareByDescending { it })) {
            val video = files.firstOrNull { it.name.contains(".stream.") }
            out.put(org.json.JSONObject().put("stamp", stamp).put("bytes", files.sumOf { it.length() })
                .put("video_bytes", video?.length() ?: 0).put("files", files.size)
                .put("uploaded", stamp in up).put("uploading", stamp == uploadingStamp)
                .put("recording", capturing && stamp == logStamp))
        }
        return out
    }

    fun deleteRecording(stamp: String): String {
        if (capturing && stamp == logStamp) return "That recording is still running."
        if (stamp == uploadingStamp) return "That recording is uploading."
        (logDir().listFiles() ?: emptyArray()).filter { stampOf(it) == stamp }.forEach { it.delete() }
        return ""
    }

    fun uploadRecording(stamp: String): String {
        if (capturing && stamp == logStamp) return "Stop recording first."
        if (uploadingStamp != null) return "Another upload is running."
        val files = (logDir().listFiles() ?: emptyArray()).filter { stampOf(it) == stamp && it.length() > 0 }
        if (files.isEmpty()) return "Nothing to upload."
        val date = "${stamp.substring(0, 4)}-${stamp.substring(4, 6)}-${stamp.substring(6, 8)}"
        uploadingStamp = stamp
        uploadFiles(files, date) { ok ->
            uploadingStamp = null
            if (ok == files.size) {
                try { uploadedFile.appendText("$stamp\n") } catch (_: Throwable) {}
                if (autoDelete) files.filter { it.name.contains(".stream.") }.forEach { it.delete() }   // the big part; logs stay (tiny)
            }
        }
        return ""
    }

    fun storage(): org.json.JSONObject {
        val d = logDir()
        return org.json.JSONObject().put("free", d.usableSpace).put("total", d.totalSpace)
            .put("used", (d.listFiles() ?: emptyArray()).sumOf { it.length() }).put("auto_delete", autoDelete)
    }

    /** Never fill the phone: refuse to start below 2 GB free, stop recording below 1 GB. */
    private fun storageGuard() {
        if (capturing && logDir().usableSpace < MIN_FREE_STOP) {
            stopCapture(); onMessage("Recording stopped: the phone is almost full. Upload and delete old videos in Videos.")
        }
    }

    private fun uploadFiles(files: List<File>, date: String, done: (Int) -> Unit = {}) {
        Thread({
            var ok = 0
            for ((i, f) in files.withIndex()) {
                val source = if (f.name.contains(".stream.")) "drone" else "telemetry"
                try {
                    if (uploadResumable(f, date, source) { sent -> uploadStatus = "Uploading ${i + 1}/${files.size} ${f.name}: ${if (f.length() > 0) sent * 100 / f.length() else 100}%" }) ok++
                } catch (t: Throwable) { event("upload ${f.name} FAILED $t") }
            }
            uploadStatus = "Uploaded $ok/${files.size} files to cradle data/raw/$date/"
            onMessage(uploadStatus)
            done(ok)
        }, "flightlab-upload").start()
    }

    /**
     * Resumable upload in 32 MB parts (dji-upload: &offset=&total=; 409/500 carry {"have": N} = where to
     * resume). Works on the tailnet and through the Cloudflare gateway (100 MB request cap). A dropped part
     * is retried up to 6 times with backoff, continuing from what the server already has.
     */
    private fun uploadResumable(f: File, date: String, source: String, progress: (Long) -> Unit): Boolean {
        val total = f.length()
        val base = "$UPLOAD_BASE/upload?date=$date&source=$source&name=${URLEncoder.encode(f.name, "UTF-8")}&total=$total"
        var offset = 0L
        var fails = 0
        java.io.RandomAccessFile(f, "r").use { raf ->
            while (true) {
                val len: Long = minOf(UPLOAD_PART, total - offset)
                val c = URL("$base&offset=$offset").openConnection() as HttpURLConnection
                c.requestMethod = "PUT"; c.doOutput = true; c.setFixedLengthStreamingMode(len)
                c.connectTimeout = 15000; c.readTimeout = 180000
                var code = -1
                var body = ""
                try {
                    raf.seek(offset)
                    c.outputStream.use { out ->
                        val buf = ByteArray(1 shl 16)
                        var left = len
                        while (left > 0) {
                            val r = raf.read(buf, 0, minOf(buf.size.toLong(), left).toInt())
                            if (r <= 0) break
                            out.write(buf, 0, r); left -= r; progress(offset + (len - left))
                        }
                    }
                    code = c.responseCode
                    val st = if (code in 200..299) c.inputStream else c.errorStream
                    body = st?.bufferedReader()?.readText() ?: ""
                } catch (t: Throwable) {
                    event("upload ${f.name} part @$offset: $t")
                } finally { c.disconnect() }
                val j: org.json.JSONObject? = try { org.json.JSONObject(body) } catch (_: Throwable) { null }
                if (code == 200 && j != null && j.optBoolean("done")) { event("upload ${f.name} done ${j.optString("path")}"); return true }
                if (code == 200 && j == null && total <= UPLOAD_PART) return true   // older server: whole file in one PUT
                val have: Long = j?.optLong("have", -1L) ?: -1L
                if (code == 200 && have >= 0) { offset = have; fails = 0; continue }
                if (have >= 0) offset = have
                fails++
                if (fails > 6) { event("upload ${f.name} gave up at $offset/$total (HTTP $code $body)"); return false }
                Thread.sleep(1000L * fails * fails)
                if (code == -1) offset = probeHave(base, total) ?: offset
            }
        }
        @Suppress("UNREACHABLE_CODE") return false
    }

    /** Where the server is with a partial upload: a 0-byte part at a wrong offset answers 409 {"have": N}. */
    private fun probeHave(base: String, total: Long): Long? = try {
        val q = URL("$base&offset=${total + 1}").openConnection() as HttpURLConnection
        q.requestMethod = "PUT"; q.doOutput = true; q.setFixedLengthStreamingMode(0L); q.connectTimeout = 15000; q.readTimeout = 30000
        q.outputStream.close()
        q.responseCode
        val h = org.json.JSONObject(q.errorStream?.bufferedReader()?.readText() ?: "{}").optLong("have", -1L)
        q.disconnect()
        if (h >= 0) h else null
    } catch (_: Throwable) { null }

    // ---- logging -------------------------------------------------------------
    fun startLogging() = synchronized(logLock) {
        if (logWriter != null) return
        val dir = File(context.getExternalFilesDir(null), "flightlab").apply { mkdirs() }
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        logStamp = stamp
        val f = File(dir, "flightlab_$stamp.csv")
        logWriter = f.bufferedWriter().apply { write(CSV_HEADER); newLine() }
        eventWriter = File(dir, "flightlab_$stamp.events.csv").bufferedWriter().apply { write("t_ns,wall_ms,event"); newLine() }
        cmdWriter = File(dir, "flightlab_$stamp.cmds.csv").bufferedWriter().apply {
            write("t_send_ns,maneuver,segment,fwd_mps,right_mps,up_mps,yaw_rate_dps,param_roll,param_pitch,param_yaw,param_vertical"); newLine()
        }
        frameWriter = File(dir, "flightlab_$stamp.video.csv").bufferedWriter().apply {
            write("t_arrival_ns,wall_ms,pts_ms,keyframe,bytes,width,height,fps"); newLine()
        }
        logFile = f
        logRows = 0
        sessionFiles.clear()
        sessionFiles += listOf(f, File(dir, "flightlab_$stamp.events.csv"), File(dir, "flightlab_$stamp.cmds.csv"), File(dir, "flightlab_$stamp.video.csv"))
        event("log_start rollIsForward=$rollIsForward maxHorizontal=$maxHorizontal cmd_hz=${1000 / CMD_PERIOD_MS} log_hz=${1000 / LOG_PERIOD_MS}")
    }

    fun stopLogging() = synchronized(logLock) {
        event("log_stop rows=$logRows")
        logWriter?.close(); eventWriter?.close(); cmdWriter?.close(); frameWriter?.close()
        logWriter = null; eventWriter = null; cmdWriter = null; frameWriter = null
        logFile = null
    }

    fun event(text: String) {
        Log.i(TAG, text)
        synchronized(logLock) {
            eventWriter?.apply {
                write("${now()},${System.currentTimeMillis()},\"${text.replace("\"", "'")}\""); newLine(); flush()
            }
        }
    }

    private fun ageMs(t: Long, n: Long) = if (t == 0L) -1 else (n - t) / 1_000_000

    private fun logTick() = synchronized(logLock) {
        val w = logWriter ?: return
        val n = now()
        val c = lastCmd
        val row = listOf(
            n, System.currentTimeMillis(), currentManeuver?.name ?: "", currentSegment,
            c.fwd, c.right, c.up, c.yawRate, if (lastCmdSent) 1 else 0,
            b(vsEnabled), b(advancedEnabled), b(latched), b(flying), b(motorsOn), b(rollIsForward),
            velN, velE, velD, ageMs(velT, n),
            pitch, roll, yaw, ageMs(attT, n),
            lat, lon, altGps, ageMs(locT, n),
            altRel, ageMs(altT, n), ultraH, ageMs(ultraT, n), ultraUsed?.let { b(it) } ?: "",
            gpsSats, rcLH, rcLV, rcRH, rcRV, battery,
            flightModeString, fcFlightMode, windWarning, gimbalPitch, ageMs(gimbalT, n), videoPackets, gimbalYaw, gimbalRoll, b(capturing), b(sdRecording)
        ).joinToString(",") { if (it is Double && it.isNaN()) "" else it.toString() }
        w.write(row); w.newLine()
        logRows++
        if (logRows % 50 == 0L) { w.flush(); cmdWriter?.flush(); frameWriter?.flush() }
    }

    private fun b(v: Boolean) = if (v) 1 else 0

    private inline fun safe(block: () -> Unit) {
        try { block() } catch (t: Throwable) { Log.e(TAG, "tick failed", t) }
    }

    companion object {
        const val TAG = "FlightLab"
        const val UPLOAD_BASE = "http://cradle.tailb1b51d.ts.net:8966"
        const val UPLOAD_PART = 32L * 1024 * 1024
        const val MIN_FREE_START = 2_000_000_000L
        const val MIN_FREE_STOP = 1_000_000_000L
        const val CMD_PERIOD_MS = 40L   // 25 Hz, MSDK's maximum send rate
        const val LOG_PERIOD_MS = 20L   // 50 Hz
        const val RC_DEADBAND = 66      // RC stick range is +/-660
        const val MAX_VERTICAL = 1.5    // m/s
        const val MAX_YAW_RATE = 90.0   // deg/s

        const val CSV_HEADER = "t_ns,wall_ms,maneuver,segment," +
            "cmd_fwd_mps,cmd_right_mps,cmd_up_mps,cmd_yaw_rate_dps,cmd_sent," +
            "vs_enabled,adv_enabled,latched,flying,motors_on,roll_is_forward," +
            "vel_n_mps,vel_e_mps,vel_d_mps,vel_age_ms," +
            "pitch_deg,roll_deg,yaw_deg,att_age_ms," +
            "lat,lon,alt_gps_m,loc_age_ms," +
            "alt_rel_m,alt_age_ms,ultra_height,ultra_age_ms,ultra_used," +
            "gps_sats,rc_lh,rc_lv,rc_rh,rc_rv,battery_pct," +
            "flight_mode,fc_flight_mode,wind_warning,gimbal_pitch_deg,gimbal_age_ms,video_packets,gimbal_yaw_deg,gimbal_roll_deg,capturing,sd_recording"

        private fun hold(s: Double, label: String = "hold") = Segment(label, s, Setpoint())

        /** Out-and-back step: hold, step to v, hard stop (step to 0), step back, stop. */
        private fun outAndBack(name: String, v: Double, axis: (Double) -> Setpoint, stepS: Double = 4.0) = Maneuver(
            name, listOf(
                hold(1.0, "pre"),
                Segment("step+${"%.0f".format(v)}", stepS, axis(v)),
                hold(3.0, "stop+"),
                Segment("step-${"%.0f".format(v)}", stepS, axis(-v)),
                hold(3.0, "stop-"),
            )
        )

        fun axisCheck() = Maneuver(
            "AXIS_CHECK", listOf(hold(1.0, "pre"), Segment("fwd0.5", 1.5, Setpoint(fwd = 0.5)), hold(2.0, "stop"))
        )

        fun forward(v: Double) = outAndBack("FWD_${v.toInt()}", v, { Setpoint(fwd = it) })
        fun lateral(v: Double) = outAndBack("LAT_${v.toInt()}", v, { Setpoint(right = it) })

        /** Direct +v -> -v reversal: separates the brake cap from the accel cap. Ends near the start. */
        fun reversal(v: Double) = Maneuver(
            "REV_${v.toInt()}", listOf(
                hold(1.0, "pre"),
                Segment("fwd+${v.toInt()}", 4.0, Setpoint(fwd = v)),
                Segment("fwd-${v.toInt()}", 4.0, Setpoint(fwd = -v)),
                hold(3.0, "stop"),
            )
        )

        /** Forward and right together: checks whether the axes couple. */
        fun diagonal(v: Double = 2.0) = Maneuver(
            "DIAG_${v.toInt()}", listOf(
                hold(1.0, "pre"),
                Segment("diag+", 4.0, Setpoint(fwd = v, right = v)), hold(3.0, "stop+"),
                Segment("diag-", 4.0, Setpoint(fwd = -v, right = -v)), hold(3.0, "stop-"),
            )
        )

        /** Yaw at 60 deg/s while flying forward at 3 m/s. A 180 deg turn each way brings it back near the start. */
        fun yawWhileMoving() = Maneuver(
            "YAW_MOVING", listOf(
                hold(1.0, "pre"),
                Segment("fwd3", 2.0, Setpoint(fwd = 3.0)),
                Segment("fwd3_yaw+60", 3.0, Setpoint(fwd = 3.0, yawRate = 60.0)),
                Segment("fwd3", 2.0, Setpoint(fwd = 3.0)),
                hold(3.0, "stop1"),
                Segment("fwd3", 2.0, Setpoint(fwd = 3.0)),
                Segment("fwd3_yaw-60", 3.0, Setpoint(fwd = 3.0, yawRate = -60.0)),
                Segment("fwd3", 2.0, Setpoint(fwd = 3.0)),
                hold(3.0, "stop2"),
            )
        )

        /** Zero command, to estimate wind drift. Run before and after the session. */
        fun hover() = Maneuver("HOVER_10", listOf(hold(10.0, "hover")))

        fun vertical() = Maneuver(
            "VERT_1", listOf(
                hold(1.0, "pre"),
                Segment("up1", 2.0, Setpoint(up = 1.0)), hold(2.5, "stop_up"),
                Segment("down1", 2.0, Setpoint(up = -1.0)), hold(2.5, "stop_down"),
            )
        )

        fun yaw() = Maneuver(
            "YAW_STEPS", listOf(hold(1.0, "pre")) + listOf(30.0, 60.0, 90.0).flatMap { r ->
                listOf(
                    Segment("yaw+${r.toInt()}", 2.0, Setpoint(yawRate = r)), hold(1.5, "stop"),
                    Segment("yaw-${r.toInt()}", 2.0, Setpoint(yawRate = -r)), hold(1.5, "stop"),
                )
            }
        )
    }
}
