package dji.sampleV5.aircraft.flightlab

import android.os.SystemClock
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen
import org.json.JSONObject
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Flies a mission saved by the web planner (missions/latest.json).
 *
 * The mission's `route_rel` is a polyline in metres east/north/up relative to the takeoff
 * spot (point A). The runner tracks it segment by segment with GPS + barometric altitude,
 * sending body-frame velocity through the Flight Lab controller, so every existing safety
 * layer still applies: any RC stick input latches and kills the mission instantly.
 *
 * States: IDLE -> LOADED -> TAKEOFF -> ARMING -> FLYING (<-> PAUSED / HOLD) -> ARRIVED -> LANDING -> DONE.
 */
class MissionRunner(private val ctl: FlightLabController, private val cacheDir: File) {

    enum class State { IDLE, LOADED, TAKEOFF, ARMING, FLYING, POLICY, PAUSED, HOLD, MANUAL, ARRIVED, LANDING, DONE, ABORTED, FOLLOW }

    data class P3(val e: Double, val n: Double, val u: Double)
    /** radius = activation range in metres (horizontal); null = always eligible. */
    data class Poi(val p: P3, val label: String, val radius: Double? = null)

    class Mission(
        val name: String, val path: List<P3>, val pois: List<Poi>, val speed: Double,
        val style: String, val fpvGimbal: Double, val takeoffLand: Boolean, val ceiling: Double,
        val corridor: Double, val homeLat: Double, val homeLon: Double, val lowBattery: Int,
        val previewS: Double, val lengthM: Double,
        val routeRel: List<P3>, val handoff: String, val text: String,
        val speeds: List<Double>? = null,   // per path point, when the mission carries a speed profile
    )

    @Volatile var state = State.IDLE; private set
    @Volatile var mission: Mission? = null; private set
    @Volatile var info = "No mission loaded."; private set
    @Volatile var holdReason = ""; private set
    @Volatile var seg = 0; private set
    @Volatile var crossTrack = 0.0; private set
    @Volatile var altError = 0.0; private set
    @Volatile var toNext = 0.0; private set
    @Volatile var progressM = 0.0; private set
    @Volatile var pos = P3(Double.NaN, Double.NaN, Double.NaN); private set
    @Volatile var activePoi = -1; private set
    @Volatile var landingNeedsConfirm = false; private set
    @Volatile var homeOffsetM = Double.NaN; private set
    var onMessage: (String) -> Unit = {}
    /** Cradle hazard/policy link; set by the activity. */
    var pilot: PilotLink? = null
    /** Called when a mission ends on the ground (for the capture upload). */
    var onLanded: () -> Unit = {}

    private var path: List<P3> = emptyList()    // mission path with a synthetic climb/descent added
    private var lat0 = Double.NaN; private var lon0 = Double.NaN
    private var cmdE = 0.0; private var cmdN = 0.0     // slew-limited horizontal command (ground frame)
    private var lastGimbalPitch = Double.NaN; private var lastGimbalNs = 0L
    private var gimbalSmooth = Double.NaN        // low-passed, rate-limited gimbal pitch target
    private var yawRateCmd = 0.0                 // slew-limited yaw rate command
    private var vLim = DoubleArray(0)            // per-vertex corner speed limit (curvature)
    private var stateNs = 0L
    @Volatile private var rejoinOnArm = false
    private val exec = Executors.newSingleThreadScheduledExecutor { Thread(it, "mission") }

    init {
        exec.scheduleAtFixedRate({ try { supervise() } catch (t: Throwable) { ctl.event("mission supervise error $t") } }, 0, 100, TimeUnit.MILLISECONDS)
        ctl.onAutopilotKilled = { why ->
            if (state == State.FLYING || state == State.POLICY || state == State.PAUSED || state == State.HOLD || state == State.FOLLOW) {
                setState(State.MANUAL, "You have control. Fly around it, centre the sticks, then tap Re-arm & continue.")
            }
        }
        FlightControllerKey.KeyIsLandingConfirmationNeeded.create().listen(this) { landingNeedsConfirm = it == true }
    }

    // ---- loading -------------------------------------------------------------------------------
    fun loadLatest() {
        if (state !in listOf(State.IDLE, State.LOADED, State.DONE, State.ABORTED)) return onMessage("Finish or abort the current mission first.")
        Thread {
            val cache = File(cacheDir, "mission_latest.json")
            val text = try {
                val c = (URL(MISSION_URL).openConnection() as HttpURLConnection).apply { connectTimeout = 8000; readTimeout = 15000 }
                val t = c.inputStream.bufferedReader().readText()
                if (c.responseCode != 200) throw IllegalStateException("HTTP ${c.responseCode}")
                cache.writeText(t); t
            } catch (e: Exception) {
                if (cache.exists()) { onMessage("Planner unreachable ($e); using the last downloaded mission."); cache.readText() }
                else { onMessage("Couldn't load a mission: $e"); return@Thread }
            }
            try {
                val m = parse(JSONObject(text), text)
                mission = m
                setState(State.LOADED, "Loaded ${m.name}: ${m.path.size} points, ${"%.0f".format(m.lengthM)} m at ${m.speed} m/s (${m.style}). Place the drone on A, then Start.")
            } catch (e: Exception) { onMessage("Mission file invalid: $e") }
        }.start()
    }

    /** Load a mission handed over directly (the in-app sandbox's "Fly for real"). Returns an error or null. */
    fun loadText(text: String): String? {
        if (state !in listOf(State.IDLE, State.LOADED, State.DONE, State.ABORTED)) return "Finish or abort the current mission first."
        return try {
            val m = parse(JSONObject(text), text)
            File(cacheDir, "mission_latest.json").writeText(text)
            mission = m
            setState(State.LOADED, "Loaded ${m.name}: ${m.path.size} points, ${"%.0f".format(m.lengthM)} m at ${m.speed} m/s (${m.style}).")
            null
        } catch (e: Exception) { "Mission invalid: ${e.message}" }
    }

    private fun parse(j: JSONObject, text: String): Mission {
        require(j.optInt("version") == 1) { "unsupported version" }
        val rr = j.getJSONArray("route_rel")
        require(rr.length() >= 2) { "route_rel needs at least 2 points" }
        val pts = (0 until rr.length()).map { rr.getJSONObject(it).let { o -> P3(o.getDouble("e_m"), o.getDouble("n_m"), o.getDouble("up_m")) } }
        // Drop duplicate consecutive points (the planner can repeat the final vertex).
        val keep = pts.indices.filter { i -> i == 0 || dist(pts[i], pts[i - 1]) > 0.05 }
        val clean = keep.map { pts[it] }
        // Optional per-point speed (e.g. a trained AI's racing line, already scaled to a safe cap).
        val sp = j.optJSONArray("speeds_mps")
        val speeds = if (sp != null && sp.length() == pts.size) keep.map { sp.optDouble(it, Double.NaN).coerceIn(0.3, MAX_SPEED) } else null
        val pr = j.optJSONArray("pois_rel")
        val pois = if (pr == null) emptyList() else (0 until pr.length()).map { pr.getJSONObject(it).let { o -> Poi(P3(o.getDouble("e_m"), o.getDouble("n_m"), o.optDouble("up_m", 1.0)), o.optString("label", "POI ${it + 1}"), o.optDouble("radius_m", Double.NaN).takeIf { r -> !r.isNaN() && r > 0 }) } }
        val lim = j.optJSONObject("limits") ?: JSONObject()
        val home = j.optJSONObject("home") ?: JSONObject()
        val sim = j.optJSONObject("sim_preview") ?: JSONObject()
        val speed = lim.optDouble("cruise_speed_mps", 3.0).coerceIn(0.5, MAX_SPEED)
        val ceiling = lim.optDouble("ceiling_m", clean.maxOf { it.u } + 5)
        require(clean.all { it.u >= 2.0 || it === clean.first() || it === clean.last() }) { "route dips below 2 m" }
        require(ceiling <= LEGAL_CEILING) { "ceiling ${ceiling} m is above the ${LEGAL_CEILING} m limit" }
        val alignment = j.optJSONObject("alignment") ?: JSONObject()
        val corridor = max(MIN_CORRIDOR, lim.optDouble("corridor_m", 1.5) + alignment.optDouble("max_margin_m", 0.0))
        return Mission(
            name = j.optString("saved_name", j.optString("name", "mission")), path = clean, pois = pois, speed = speed,
            style = j.optString("flight_style", "normal"), fpvGimbal = j.optDouble("fpv_gimbal_deg", -10.0),
            takeoffLand = j.optBoolean("takeoff_land", true), ceiling = ceiling, corridor = corridor,
            homeLat = home.optDouble("lat", Double.NaN), homeLon = home.optDouble("lon", Double.NaN),
            lowBattery = (j.optJSONObject("failsafe")?.optInt("on_low_battery_pct", 30) ?: 30).coerceAtLeast(25),
            previewS = sim.optDouble("duration_s", Double.NaN),
            lengthM = clean.zipWithNext { a, b -> dist(a, b) }.sum(),
            routeRel = pts, handoff = j.optString("obstacle_handoff", "human"), text = text, speeds = speeds,
        )
    }

    // ---- commands from the UI ------------------------------------------------------------------
    fun start() {
        val m = mission ?: return onMessage("Load a mission first.")
        if (state != State.LOADED && state != State.DONE && state != State.ABORTED) return onMessage("Mission already running.")
        preflight()?.let { return onMessage("Can't start: $it") }
        // Origin = the takeoff spot. On the ground that's where we are now; in the air it's DJI's home point.
        if (ctl.flying) {
            val h = FlightControllerKey.KeyHomeLocation.create().get()
            if (h == null) return onMessage("Can't start in the air: no home point. Land on A and start from the ground.")
            lat0 = h.latitude; lon0 = h.longitude
        } else { lat0 = ctl.lat; lon0 = ctl.lon }
        homeOffsetM = if (m.homeLat.isNaN()) Double.NaN else enu(m.homeLat, m.homeLon).let { hypot(it.first, it.second) }
        // Start from where the drone actually is, then join the route at A.
        val here = P3(0.0, 0.0, max(ctl.altRel.takeIf { !it.isNaN() } ?: 0.0, if (ctl.flying) 0.0 else TAKEOFF_ALT))
        path = buildList {
            add(P3(0.0, 0.0, here.u)); addAll(m.path)
            if (m.takeoffLand) m.path.last().let { add(P3(it.e, it.n, LAND_APPROACH_ALT)) }
        }.let { l -> l.filterIndexed { i, p -> i == 0 || dist(p, l[i - 1]) > 0.05 } }.let { smoothCorners(it) }
        vLim = cornerLimits(path, m)
        seg = 0; progressM = 0.0; cmdE = 0.0; cmdN = 0.0; activePoi = -1; lastGimbalPitch = Double.NaN; gimbalSmooth = Double.NaN; yawRateCmd = 0.0
        if (ctl.logFile == null) ctl.startLogging()
        // Every mission also records the full stream + 50 Hz telemetry (+ SD 4K) for the data flywheel.
        if (!ctl.capturing) ctl.startCapture(recordSd = true)
        pilot?.let { if (it.mode != PilotLink.Mode.OFF) it.connect(m.text) }
        ctl.event("mission_start ${m.name} origin=$lat0,$lon0 home_offset_m=$homeOffsetM pts=${path.size}")
        if (!homeOffsetM.isNaN() && homeOffsetM > HOME_WARN_M) onMessage("Warning: you're ${"%.0f".format(homeOffsetM)} m from the planned A. The route is flown relative to here.")
        if (ctl.flying) arm() else {
            setState(State.TAKEOFF, "Taking off…")
            FlightControllerKey.KeyStartTakeoff.create().action({ ctl.event("takeoff ok") }, { e -> setState(State.ABORTED, "Takeoff failed: $e") })
        }
    }

    fun pause() {
        when (state) {
            State.MANUAL -> continueAfterManual()
            State.FLYING, State.POLICY -> setState(State.PAUSED, "Paused, hovering. Resume to continue.")
            State.PAUSED, State.HOLD -> { preflight(inFlight = true, checkTrack = false)?.let { return onMessage("Can't resume: $it") }; cmdE = 0.0; cmdN = 0.0; setState(State.FLYING, "Resumed.") }
            else -> onMessage("Nothing to pause.")
        }
    }

    /** After the pilot flew around something: re-arm, retake control and rejoin the route at the nearest point ahead. */
    fun continueAfterManual() {
        if (state != State.MANUAL) return onMessage("Nothing to continue.")
        if (!ctl.sticksCentered()) return onMessage("Centre the RC sticks first.")
        if (ctl.latched) ctl.rearm()
        if (ctl.latched) return
        if (following) { followCheck()?.let { return onMessage("Can't follow: $it") }; arm(); return }
        preflight(inFlight = true, checkTrack = false)?.let { return onMessage("Can't continue: $it") }
        rejoinOnArm = true
        arm()
    }

    /** Index of the route_rel vertex after the route segment closest to us (never behind our progress). */
    private fun nearestAhead(): Int {
        val m = mission ?: return 0
        val p = posRel(); val r = m.routeRel
        var best = 0; var bestD = Double.MAX_VALUE
        for (i in 0 until r.size - 1) {
            val a = r[i]; val b = r[i + 1]; val L = dist(a, b)
            val s = along(a, b, p).coerceIn(0.0, L)
            val c = if (L < 1e-6) a else P3(a.e + (b.e - a.e) / L * s, a.n + (b.n - a.n) / L * s, a.u + (b.u - a.u) / L * s)
            val d = dist(c, p)
            if (d < bestD) { bestD = d; best = i + 1 }
        }
        return best
    }

    /** Stop the mission and hover under app control; the pilot can then disable VS or fly. */
    fun abort(why: String = "abort pressed") {
        if (state in listOf(State.IDLE, State.LOADED, State.DONE, State.ABORTED)) return
        rejoinOnArm = false
        following = false
        ctl.autopilot = null
        if (state == State.LANDING) FlightControllerKey.KeyStopAutoLanding.create().action({}, {})
        setState(State.ABORTED, "Mission aborted ($why). Hovering; take over with the RC or Disable VS.")
    }

    fun land() {
        if (!ctl.flying) return onMessage("Not flying.")
        following = false
        ctl.autopilot = null
        setState(State.LANDING, "Landing…")
        if (ctl.vsEnabled) ctl.disableVirtualStick()
        FlightControllerKey.KeyStartAutoLanding.create().action({ ctl.event("auto landing started") }, { e -> onMessage("Landing failed: $e. Land with the RC.") })
    }

    fun confirmLanding() = FlightControllerKey.KeyConfirmLanding.create().action({ ctl.event("landing confirmed") }, { e -> onMessage("Confirm failed: $e") })

    fun shutdown() { abort("screen closed"); exec.shutdownNow(); dji.v5.manager.KeyManager.getInstance().cancelListen(this) }

    // ---- state machine (10 Hz) -----------------------------------------------------------------
    private fun supervise() {
        if (following) return superviseFollow()
        val m = mission ?: return
        when (state) {
            State.TAKEOFF -> when {
                ctl.latched -> setState(State.ABORTED, "RC took over during takeoff.")
                ctl.flying && ctl.altRel > 0.8 -> arm()
                elapsed() > 20 -> setState(State.ABORTED, "Takeoff timed out.")
            }
            State.ARMING -> when {
                ctl.vsEnabled && ctl.advancedEnabled -> {
                    ctl.autopilotLabel = "seg 0"; ctl.autopilot = { control(m) }
                    if (rejoinOnArm) { rejoinOnArm = false; rejoin(nearestAhead()) }
                    else setState(State.FLYING, "Flying ${m.name}. Any RC stick input takes over.")
                }
                ctl.latched -> setState(State.ABORTED, "RC override latched; Re-arm and start again.")
                elapsed() > 6 -> setState(State.ABORTED, "Couldn't enable virtual stick.")
            }
            State.FLYING, State.POLICY -> preflight(inFlight = true)?.let { hold(it) }
            State.ARRIVED -> if (m.takeoffLand && elapsed() > 1.5) land()
            State.LANDING -> if (!ctl.flying && !ctl.motorsOn) { setState(State.DONE, "Landed. Mission complete. Uploading the recording…"); pilot?.close(); onLanded() }
            else -> {}
        }
    }

    private fun arm() {
        setState(State.ARMING, "Handing control to the app…")
        ctl.maxHorizontal = mission?.speed ?: 3.0
        if (!(ctl.vsEnabled && ctl.advancedEnabled)) ctl.enableVirtualStick()
    }

    /** Safety checks; returns a reason to refuse/hold, or null when OK. */
    private fun preflight(inFlight: Boolean = false, checkTrack: Boolean = true): String? {
        val n = SystemClock.elapsedRealtimeNanos()
        val m = mission ?: return "no mission"
        return when {
            ctl.latched -> "RC override is latched. Centre sticks and Re-arm."
            !ctl.sticksCentered() -> "RC sticks aren't centred."
            ctl.lat.isNaN() -> "no GPS fix yet (${ctl.gpsSats} satellites). Wait outdoors with a clear sky view."
            (n - ctl.locT) > 1_500_000_000L -> "no fresh GPS position (last fix ${(n - ctl.locT) / 1_000_000_000L} s ago, ${ctl.gpsSats} satellites)"
            ctl.gpsSats in 0 until MIN_SATS -> "only ${ctl.gpsSats} GPS satellites (need $MIN_SATS)"
            ctl.altRel.isNaN() || (n - ctl.altT) > 1_500_000_000L -> "no fresh altitude (last ${(n - ctl.altT) / 1_000_000_000L} s ago)"
            ctl.battery in 0 until m.lowBattery -> "battery ${ctl.battery}% is below ${m.lowBattery}%"
            inFlight && ctl.altRel > m.ceiling + 3 -> "above the mission ceiling (${"%.0f".format(ctl.altRel)} m)"
            inFlight && checkTrack && state != State.POLICY && abs(altError) > ALT_TOLERANCE -> "${"%.1f".format(altError)} m off the planned height"
            inFlight && checkTrack && abs(crossTrack) > corridorLimit(m) -> "${"%.1f".format(abs(crossTrack))} m off the planned path (limit ${"%.1f".format(corridorLimit(m))} m)"
            else -> null
        }
    }

    private fun corridorLimit(m: Mission) = if (state == State.POLICY) max(POLICY_CORRIDOR, m.corridor + CORRIDOR_SLACK) else m.corridor + CORRIDOR_SLACK

    private fun hold(why: String) {
        holdReason = why
        setState(State.HOLD, "HOLDING: $why. Fix it and Resume, or take over with the RC.")
    }

    // ---- 25 Hz path tracking (runs on the Flight Lab control thread) ----------------------------
    private fun control(m: Mission): FlightLabController.Setpoint {
        val (e, n) = enu(ctl.lat, ctl.lon)
        val p = P3(e, n, ctl.altRel)
        pos = p

        // Advance past segments we've reached (only while actually flying the route).
        val fpv = m.style == "fpv"
        while (state == State.FLYING && seg < path.size - 1) {
            val a = path[seg]; val b = path[seg + 1]
            val L = dist(a, b); val s = along(a, b, p)
            val switchAt = if (fpv && seg + 2 < path.size && turnDeg(seg + 1) < 60) FPV_SWITCH_M else ARRIVE_M
            if (L - s < switchAt && dist(p, b) < switchAt + 2.0) { progressM += L; seg++; ctl.autopilotLabel = "seg $seg"; ctl.event("mission_vertex $seg") } else break
        }
        if (seg >= path.size - 1) {
            if (state == State.FLYING) setState(State.ARRIVED, if (m.takeoffLand) "Arrived. Landing…" else "Arrived. Hovering at the end point; land with the RC or tap Land.")
            cmdE = 0.0; cmdN = 0.0
            return FlightLabController.Setpoint()
        }
        val a = path[seg]; val b = path[seg + 1]
        val L = dist(a, b); val s = along(a, b, p).coerceIn(0.0, L)
        val dE = (b.e - a.e) / L; val dN = (b.n - a.n) / L; val dU = (b.u - a.u) / L
        val c = P3(a.e + dE * s, a.n + dN * s, a.u + dU * s)
        val xE = p.e - c.e; val xN = p.n - c.n; val xU = p.u - c.u
        crossTrack = hypot(xE, xN)
        toNext = L - s
        altError = xU
        if (state == State.POLICY) {
            // The learned dodge policy flies; hover if its commands go stale (300 ms) or it stops asking.
            val c = pilot?.freshCmd(300)
            cmdE = ctl.velE.takeIf { !it.isNaN() } ?: 0.0; cmdN = ctl.velN.takeIf { !it.isNaN() } ?: 0.0
            if (c == null || c.want != "policy") return FlightLabController.Setpoint()
            val h = hypot(c.fwd, c.right); val k = if (h > POLICY_MAX_SPEED) POLICY_MAX_SPEED / h else 1.0
            return FlightLabController.Setpoint(fwd = c.fwd * k, right = c.right * k, up = c.up, yawRate = c.yawRate.coerceIn(-MAX_YAW, MAX_YAW))
        }
        if (state != State.FLYING) { cmdE = 0.0; cmdN = 0.0; return FlightLabController.Setpoint() }

        // Along-track speed: cruise, slowing for turns and for the final point.
        val horiz = hypot(b.e - a.e, b.n - a.n)
        val vertical = horiz < 0.5
        val cruise = if (vertical) VERTICAL_SPEED else m.speed
        // Brake early enough for the tightest corner (or the end) within stopping distance.
        var v = cruise; var ahead = L - s
        for (k in seg + 1 until path.size) {
            v = min(v, sqrt(vLim[k] * vLim[k] + 2 * DECEL * ahead))
            if (ahead > cruise * cruise / (2 * DECEL) + 1 || k + 1 >= path.size) break
            ahead += dist(path[k], path[k + 1])
        }
        // Velocity = along-track + cross-track correction toward the line.
        var vE = dE * v - K_CROSS * xE
        var vN = dN * v - K_CROSS * xN
        val vU = (dU * v - K_ALT * xU).coerceIn(-MAX_DESCENT, FlightLabController.MAX_VERTICAL)
        val hv = hypot(vE, vN)
        if (hv > m.speed) { vE *= m.speed / hv; vN *= m.speed / hv }
        // Acceleration limit (smooth starts and stops).
        val dt = FlightLabController.CMD_PERIOD_MS / 1000.0
        val step = ACCEL * dt
        cmdE += (vE - cmdE).coerceIn(-step, step); cmdN += (vN - cmdN).coerceIn(-step, step)

        // Heading: look at the active POI (normal style) or at a point a little way ahead on the path,
        // so vertices don't kick the yaw; the yaw rate itself is slew-limited.
        val la = aheadPoint(c, LOOKAHEAD_MIN + LOOKAHEAD_S * hypot(cmdE, cmdN))
        val laH = hypot(la.e - p.e, la.n - p.n)
        val targetYaw = poiYaw(m, p) ?: if (horiz > 1.0 && laH > 1.0) Math.toDegrees(atan2(la.e - p.e, la.n - p.n)) else if (horiz > 1.0) Math.toDegrees(atan2(dE, dN)) else ctl.yaw
        val yawRate = smoothYawRate(wrap(targetYaw - ctl.yaw), dt)
        gimbal(m, p)

        // Ground frame -> body frame (yaw 0 = north, clockwise positive).
        val y = Math.toRadians(ctl.yaw)
        val fwd = cmdN * cos(y) + cmdE * sin(y)
        val right = -cmdN * sin(y) + cmdE * cos(y)
        return FlightLabController.Setpoint(fwd = fwd, right = right, up = vU, yawRate = yawRate)
    }

    private fun poiYaw(m: Mission, p: P3): Double? {
        if (m.pois.isEmpty() || m.style == "fpv") return null
        // Nearest POI within its range, with 2 m hysteresis like the sim. Outside every range: face ahead.
        val d = m.pois.map { hypot(it.p.e - p.e, it.p.n - p.n) }
        fun inRange(i: Int, slack: Double) = m.pois[i].radius?.let { d[i] <= it + slack } ?: true
        if (activePoi >= 0 && !inRange(activePoi, POI_HYST)) activePoi = -1
        val best = d.indices.filter { inRange(it, 0.0) }.minByOrNull { d[it] }
        if (best != null && (activePoi < 0 || d[best] + POI_HYST < d[activePoi])) activePoi = best
        if (activePoi < 0) return null
        val q = m.pois[activePoi].p
        return Math.toDegrees(atan2(q.e - p.e, q.n - p.n))
    }

    private fun gimbal(m: Mission, p: P3) {
        val target = if (activePoi >= 0 && m.style != "fpv") {
            val q = m.pois[activePoi].p
            Math.toDegrees(atan2(q.u - p.u, hypot(q.e - p.e, q.n - p.n))).coerceIn(-90.0, 20.0)
        } else if (m.style == "fpv") m.fpvGimbal else DEFAULT_GIMBAL
        sendGimbal(target)
    }

    /** Glide the gimbal: low-pass + rate-limit the target at 25 Hz, send small absolute moves every 200 ms. */
    private fun sendGimbal(target: Double) {
        val dt = FlightLabController.CMD_PERIOD_MS / 1000.0
        if (gimbalSmooth.isNaN()) gimbalSmooth = ctl.gimbalPitch.takeIf { !it.isNaN() } ?: target
        val want = gimbalSmooth + (target - gimbalSmooth) * min(1.0, dt / GIMBAL_TAU)
        gimbalSmooth += (want - gimbalSmooth).coerceIn(-GIMBAL_RATE * dt, GIMBAL_RATE * dt)
        val now = SystemClock.elapsedRealtimeNanos()
        if (!lastGimbalPitch.isNaN() && (abs(gimbalSmooth - lastGimbalPitch) < 0.4 || now - lastGimbalNs < 200_000_000L)) return
        lastGimbalPitch = gimbalSmooth; lastGimbalNs = now
        val r = GimbalAngleRotation().apply {
            mode = GimbalAngleRotationMode.ABSOLUTE_ANGLE; pitch = gimbalSmooth; yaw = 0.0; roll = 0.0
            pitchIgnored = false; yawIgnored = true; rollIgnored = true; duration = 0.25
        }
        GimbalKey.KeyRotateByAngle.create().action(r, {}, { e -> ctl.event("gimbal rotate failed $e") })
    }

    /** P on heading error, then limit how fast the commanded yaw rate may change (no snap turns). */
    private fun smoothYawRate(errDeg: Double, dt: Double): Double {
        val want = if (abs(errDeg) < YAW_DEADBAND) 0.0 else (K_YAW * errDeg).coerceIn(-MAX_YAW, MAX_YAW)
        yawRateCmd += (want - yawRateCmd).coerceIn(-YAW_ACC * dt, YAW_ACC * dt)
        return yawRateCmd
    }

    /** The point `d` metres further along the path from c (c lies on segment `seg`). */
    private fun aheadPoint(c: P3, d: Double): P3 {
        var left = d; var from = c; var k = seg + 1
        while (k < path.size) {
            val to = path[k]; val L = dist(from, to)
            if (L >= left) { val f = left / L; return P3(from.e + (to.e - from.e) * f, from.n + (to.n - from.n) * f, from.u + (to.u - from.u) * f) }
            left -= L; from = to; k++
        }
        return path.last()
    }

    /**
     * Round horizontal corners: three passes of bounded corner cutting (each cut <= 25% of the leg and
     * <= 1.5, 0.75, 0.4 m), so the drone sweeps through instead of stopping and pivoting. Climbs and
     * descents in place (horizontal leg < 0.5 m) and the end points are kept exactly.
     */
    private fun smoothCorners(pts: List<P3>): List<P3> {
        var cur = pts
        for (cut in doubleArrayOf(1.5, 0.75, 0.4)) {
            if (cur.size < 3) return cur
            val out = ArrayList<P3>(cur.size * 2); out += cur.first()
            for (i in 1 until cur.size - 1) {
                val a = cur[i - 1]; val b = cur[i]; val c = cur[i + 1]
                val h1 = hypot(b.e - a.e, b.n - a.n); val h2 = hypot(c.e - b.e, c.n - b.n)
                if (h1 < 0.5 || h2 < 0.5 || turnAt(a, b, c) < 4.0) { out += b; continue }
                val f1 = min(0.25, cut / dist(a, b)); val f2 = min(0.25, cut / dist(b, c))
                out += P3(b.e + (a.e - b.e) * f1, b.n + (a.n - b.n) * f1, b.u + (a.u - b.u) * f1)
                out += P3(b.e + (c.e - b.e) * f2, b.n + (c.n - b.n) * f2, b.u + (c.u - b.u) * f2)
            }
            out += cur.last(); cur = out
        }
        return cur
    }

    private fun turnAt(a: P3, b: P3, c: P3): Double {
        val h1 = hypot(b.e - a.e, b.n - a.n); val h2 = hypot(c.e - b.e, c.n - b.n)
        if (h1 < 1e-6 || h2 < 1e-6) return 90.0
        val d = ((b.e - a.e) * (c.e - b.e) + (b.n - a.n) * (c.n - b.n)) / (h1 * h2)
        return Math.toDegrees(kotlin.math.acos(d.coerceIn(-1.0, 1.0)))
    }

    /** Speed allowed through each vertex: sqrt(lateral accel * turn radius); 0 at the end, slow at climbs in place. */
    private fun cornerLimits(pts: List<P3>, m: Mission): DoubleArray = curvatureLimits(pts, m).also { lim ->
        val sp = m.speeds ?: return@also
        // Speed profile: each (smoothed) vertex takes the speed of the nearest mission point, searched forward.
        var j = 0
        for (i in 1 until pts.size - 1) {
            while (j + 1 < m.path.size && dist(pts[i], m.path[j + 1]) <= dist(pts[i], m.path[j])) j++
            lim[i] = min(lim[i], sp[j])
        }
    }

    private fun curvatureLimits(pts: List<P3>, m: Mission): DoubleArray = DoubleArray(pts.size) { i ->
        when {
            i == 0 -> m.speed
            i == pts.size - 1 -> 0.0
            else -> {
                val a = pts[i - 1]; val b = pts[i]; val c = pts[i + 1]
                val h1 = hypot(b.e - a.e, b.n - a.n); val h2 = hypot(c.e - b.e, c.n - b.n)
                if (h1 < 0.5 || h2 < 0.5) CORNER_SPEED else {
                    val th = Math.toRadians(turnAt(a, b, c))
                    if (th < 1e-3) m.speed else min(m.speed, max(CORNER_SPEED, sqrt(A_LAT * (min(h1, h2) / 2) / kotlin.math.tan(th / 2))))
                }
            }
        }
    }

    // ---- Follow me: hold a set distance and height from the phone, nose and camera on it ----------
    @Volatile var following = false; private set
    @Volatile var followInfo = ""; private set
    private var followDist = 8.0; private var followHeight = 6.0
    private var tgtE = Double.NaN; private var tgtN = Double.NaN

    fun startFollow(distM: Double, heightM: Double) {
        if (state !in listOf(State.IDLE, State.LOADED, State.DONE, State.ABORTED)) return onMessage("Stop the current flight first.")
        if (ctl.latched) return onMessage("RC override is latched. Centre sticks and Re-arm.")
        if (!ctl.sticksCentered()) return onMessage("Centre the RC sticks first.")
        val n = SystemClock.elapsedRealtimeNanos()
        if (ctl.lat.isNaN() || n - ctl.locT > 1_500_000_000L) return onMessage("No drone GPS yet.")
        if (ctl.gpsSats in 0 until MIN_SATS) return onMessage("Only ${ctl.gpsSats} drone GPS satellites (need $MIN_SATS).")
        if (PhoneGps.ageS() > 3 || !(PhoneGps.acc <= FOLLOW_MAX_ACC)) return onMessage("Waiting for a good phone GPS fix (±${FOLLOW_MAX_ACC.toInt()} m). Go outside, allow location.")
        if (ctl.battery in 0 until FOLLOW_MIN_BATTERY) return onMessage("Battery ${ctl.battery}% is too low to follow.")
        followDist = distM.coerceIn(FOLLOW_MIN_DIST, 30.0); followHeight = heightM.coerceIn(FOLLOW_MIN_HEIGHT, 40.0)
        following = true; tgtE = Double.NaN; tgtN = Double.NaN; cmdE = 0.0; cmdN = 0.0; yawRateCmd = 0.0; gimbalSmooth = Double.NaN; lastGimbalPitch = Double.NaN
        // Origin: home point in the air, here on the ground; the range limit is measured from it.
        if (ctl.flying) {
            val h = FlightControllerKey.KeyHomeLocation.create().get()
            if (h != null) { lat0 = h.latitude; lon0 = h.longitude } else { lat0 = ctl.lat; lon0 = ctl.lon }
        } else { lat0 = ctl.lat; lon0 = ctl.lon }
        if (ctl.logFile == null) ctl.startLogging()
        ctl.event("follow_start dist=$followDist height=$followHeight")
        if (ctl.flying) arm() else {
            setState(State.TAKEOFF, "Taking off to follow you…")
            FlightControllerKey.KeyStartTakeoff.create().action({ ctl.event("takeoff ok") }, { e -> following = false; setState(State.ABORTED, "Takeoff failed: $e") })
        }
    }

    fun stopFollow() { if (following) abort("follow stopped") }

    private fun followCheck(): String? = when {
        ctl.battery in 0 until FOLLOW_MIN_BATTERY -> "battery ${ctl.battery}%: land now"
        ctl.lat.isNaN() || SystemClock.elapsedRealtimeNanos() - ctl.locT > 2_000_000_000L -> "lost the drone's GPS"
        else -> null
    }

    private fun superviseFollow() {
        when (state) {
            State.TAKEOFF -> when {
                ctl.latched -> { following = false; setState(State.ABORTED, "RC took over during takeoff.") }
                ctl.flying && ctl.altRel > 0.8 -> arm()
                elapsed() > 20 -> { following = false; setState(State.ABORTED, "Takeoff timed out.") }
            }
            State.ARMING -> when {
                ctl.vsEnabled && ctl.advancedEnabled -> {
                    ctl.autopilotLabel = "follow"; ctl.autopilot = { followControl() }
                    setState(State.FOLLOW, "Following you at ${followDist.toInt()} m, ${followHeight.toInt()} m up. No obstacle sensing: keep it in the open. Any RC stick input takes over.")
                }
                ctl.latched -> { following = false; setState(State.ABORTED, "RC override latched; Re-arm and start again.") }
                elapsed() > 6 -> { following = false; setState(State.ABORTED, "Couldn't enable virtual stick.") }
            }
            State.FOLLOW -> followCheck()?.let { abort(it) }
            else -> {}
        }
    }

    private fun followControl(): FlightLabController.Setpoint {
        val dt = FlightLabController.CMD_PERIOD_MS / 1000.0
        val (e, n) = enu(ctl.lat, ctl.lon)
        val p = P3(e, n, ctl.altRel); pos = p
        val vU = (K_ALT * (followHeight - ctl.altRel)).coerceIn(-MAX_DESCENT, FlightLabController.MAX_VERTICAL)
        val fresh = PhoneGps.ageS() < FOLLOW_STALE_S && PhoneGps.acc <= FOLLOW_MAX_ACC
        val step = ACCEL * dt
        if (!fresh) {   // lost the phone: brake to a hover and keep height; resumes when the fix is back
            followInfo = "Waiting for the phone's GPS · hovering"
            cmdE += (0 - cmdE).coerceIn(-step, step); cmdN += (0 - cmdN).coerceIn(-step, step)
            return bodySetpoint(vU, smoothYawRate(0.0, dt))
        }
        val (pe, pn) = enu(PhoneGps.lat, PhoneGps.lon)
        val moving = PhoneGps.speed > 0.8 && !PhoneGps.bearing.isNaN()
        val b = Math.toRadians(PhoneGps.bearing)
        val pvE = if (moving) PhoneGps.speed * sin(b) else 0.0; val pvN = if (moving) PhoneGps.speed * cos(b) else 0.0
        // Where to be: behind you when you walk; otherwise stay on the side it's already on, at the set distance.
        val dx = e - pe; val dy = n - pn; val dh = hypot(dx, dy)
        val (ux, uy) = if (moving) Pair(-sin(b), -cos(b)) else if (dh > 0.5) Pair(dx / dh, dy / dh) else Pair(0.0, -1.0)
        var te = pe + ux * followDist; var tn = pn + uy * followDist
        val r = hypot(te, tn); if (r > FOLLOW_RANGE) { te *= FOLLOW_RANGE / r; tn *= FOLLOW_RANGE / r }   // leash from the start point
        // Low-pass the target: phone GPS jitters by metres; the drone shouldn't.
        val k = min(1.0, dt / FOLLOW_TAU)
        if (tgtE.isNaN()) { tgtE = te; tgtN = tn } else { tgtE += (te - tgtE) * k; tgtN += (tn - tgtN) * k }
        var vE = K_FOLLOW * (tgtE - e) + pvE; var vN = K_FOLLOW * (tgtN - n) + pvN
        if (dh < FOLLOW_MIN_DIST && dh > 0.1) { vE += (dx / dh) * 1.0; vN += (dy / dh) * 1.0 }   // never closer than 3 m
        val hv = hypot(vE, vN); if (hv > FOLLOW_MAX_SPEED) { vE *= FOLLOW_MAX_SPEED / hv; vN *= FOLLOW_MAX_SPEED / hv }
        if (hypot(tgtE - e, tgtN - n) < 0.7 && !moving) { vE = 0.0; vN = 0.0 }   // close enough: don't hunt
        cmdE += (vE - cmdE).coerceIn(-step, step); cmdN += (vN - cmdN).coerceIn(-step, step)
        // Nose and camera on the phone (held about 1.2 m above the takeoff ground).
        val yawRate = if (dh > 1.5) smoothYawRate(wrap(Math.toDegrees(atan2(pe - e, pn - n)) - ctl.yaw), dt) else smoothYawRate(0.0, dt)
        sendGimbal((-Math.toDegrees(atan2(ctl.altRel - 1.2, max(dh, 0.5)))).coerceIn(-90.0, 0.0))
        followInfo = "Following · ${"%.0f".format(dh)} m from you · phone GPS ±${"%.0f".format(PhoneGps.acc)} m"
        return bodySetpoint(vU, yawRate)
    }

    private fun bodySetpoint(vU: Double, yawRate: Double): FlightLabController.Setpoint {
        val y = Math.toRadians(ctl.yaw)
        return FlightLabController.Setpoint(fwd = cmdN * cos(y) + cmdE * sin(y), right = -cmdN * sin(y) + cmdE * cos(y), up = vU, yawRate = yawRate)
    }

    // ---- helpers -------------------------------------------------------------------------------
    private fun setState(s: State, msg: String) {
        if (s != State.HOLD) holdReason = ""
        state = s; info = msg; stateNs = SystemClock.elapsedRealtimeNanos()
        ctl.event("mission_state $s: $msg")
        onMessage(msg)
    }

    private fun elapsed() = (SystemClock.elapsedRealtimeNanos() - stateNs) / 1e9

    fun enu(lat: Double, lon: Double): Pair<Double, Double> {
        val n = (lat - lat0) * M_PER_DEG_LAT
        val e = (lon - lon0) * M_PER_DEG_LAT * cos(Math.toRadians(lat0))
        return e to n
    }

    private fun turnDeg(i: Int): Double {
        if (i <= 0 || i >= path.size - 1) return 0.0
        val a = path[i - 1]; val b = path[i]; val c = path[i + 1]
        val h1 = hypot(b.e - a.e, b.n - a.n); val h2 = hypot(c.e - b.e, c.n - b.n)
        if (h1 < 0.5 || h2 < 0.5) return 90.0   // a climb or descent in place: treat as a full stop
        val d = ((b.e - a.e) * (c.e - b.e) + (b.n - a.n) * (c.n - b.n)) / (h1 * h2)
        return Math.toDegrees(kotlin.math.acos(d.coerceIn(-1.0, 1.0)))
    }

    /** Remaining path length from the current position, for the progress readout. */
    fun remainingM(): Double {
        if (path.isEmpty()) return 0.0
        return toNext + path.drop(seg + 1).zipWithNext { a, b -> dist(a, b) }.sum()
    }

    fun pathPoints(): List<P3> = path

    /** Position in metres east/north/up from the takeoff spot (NaN until a mission starts). */
    fun posRel(): P3 {
        if (lat0.isNaN() || ctl.lat.isNaN()) return P3(Double.NaN, Double.NaN, Double.NaN)
        val (e, n) = enu(ctl.lat, ctl.lon); return P3(e, n, ctl.altRel)
    }

    /** Who is really flying, for the pilot protocol. */
    fun flyingBy(): String = when {
        ctl.latched || !ctl.vsEnabled -> "rc"
        state == State.FLYING -> "mission"
        state == State.POLICY -> if (pilot?.freshCmd(300)?.want == "policy") "policy" else "hover"
        else -> "hover"
    }

    /** Pilot server changed its mind (called on the network thread). */
    fun onPilotWant(c: PilotLink.Cmd) {
        val m = mission ?: return
        val apply = pilot?.mode == PilotLink.Mode.POLICY
        when (c.want) {
            "policy" -> when {
                state != State.FLYING -> {}
                !apply -> onMessage("SHADOW: the policy would take over here (p_brake ${"%.2f".format(c.pBrake)}, ${c.reason}).")
                m.handoff == "policy" -> setState(State.POLICY, "Obstacle ahead: the learned policy is flying (${c.reason}). RC stick = take over.")
                else -> hold("obstacle ahead (${c.reason}); this mission hands obstacles to you, take over with the RC")
            }
            "mission" -> if (state == State.POLICY) rejoin(c.rejoinIdx)
            "hover" -> when {
                !apply -> onMessage("SHADOW: the server would hover here (${c.reason}).")
                state == State.FLYING || state == State.POLICY -> hold("pilot server asked to hover (${c.reason})")
            }
        }
    }

    fun onPilotLost(why: String) {
        if (state == State.POLICY) hold("pilot link lost while the policy was flying ($why)")
        else if (state == State.FLYING) onMessage("Pilot link lost ($why); still flying the mission on GPS.")
    }

    /** Resume the route at route_rel[idx], flying straight there from where the dodge left us. */
    private fun rejoin(idx: Int?) {
        val m = mission ?: return
        val i = (idx ?: 0).coerceIn(0, m.routeRel.size - 1)
        val here = posRel()
        path = buildList {
            add(here); addAll(m.routeRel.drop(i))
            if (m.takeoffLand) m.routeRel.last().let { add(P3(it.e, it.n, LAND_APPROACH_ALT)) }
        }.let { l -> l.filterIndexed { k, p -> k == 0 || dist(p, l[k - 1]) > 0.05 } }
        seg = 0; ctl.autopilotLabel = "rejoin $i"
        ctl.event("mission_rejoin route_idx=$i from=${here}")
        setState(State.FLYING, "Path clear: back on the mission at point ${i + 1}.")
    }
    fun ctlLabel() = "point ${seg + 1}/${max(1, path.size - 1)}"

    companion object {
        const val MISSION_URL = "https://cradle.tailb1b51d.ts.net:8444/api/missions/latest"
        const val MAX_SPEED = 8.0          // m/s, hard cap regardless of the mission file
        const val LEGAL_CEILING = 120.0    // m AGL (FAA Part 107 / recreational limit)
        const val MIN_SATS = 10
        const val MIN_CORRIDOR = 3.0       // m; GPS alone wanders 1-2 m
        const val CORRIDOR_SLACK = 2.0
        const val ALT_TOLERANCE = 3.0
        const val HOME_WARN_M = 15.0
        const val TAKEOFF_ALT = 1.2        // DJI auto takeoff hovers about here
        const val LAND_APPROACH_ALT = 3.0
        const val ARRIVE_M = 1.0
        const val FPV_SWITCH_M = 3.0
        const val VERTICAL_SPEED = 1.0
        const val CORNER_SPEED = 0.5
        const val DECEL = 1.2              // m/s^2 when slowing for a corner
        const val ACCEL = 1.5              // m/s^2 command slew
        const val K_CROSS = 0.8            // 1/s
        const val K_ALT = 0.8              // 1/s
        const val MAX_DESCENT = 1.0        // m/s
        const val K_YAW = 1.5              // deg/s per deg
        const val MAX_YAW = 60.0           // deg/s
        const val YAW_ACC = 90.0           // deg/s^2: how fast the yaw rate may change
        const val YAW_DEADBAND = 1.5       // deg
        const val LOOKAHEAD_MIN = 3.0      // m: heading aims this far ahead on the path...
        const val LOOKAHEAD_S = 1.0        // ...plus 1 s of travel
        const val A_LAT = 1.5              // m/s^2 sideways accel allowed through a corner
        const val GIMBAL_TAU = 0.6         // s, gimbal target low-pass
        const val GIMBAL_RATE = 30.0       // deg/s max gimbal slew
        const val K_FOLLOW = 0.6           // 1/s position gain toward the follow spot
        const val FOLLOW_TAU = 1.0         // s, phone-position low-pass
        const val FOLLOW_MAX_SPEED = 6.0   // m/s
        const val FOLLOW_MIN_DIST = 3.0    // m from the phone, never closer
        const val FOLLOW_MIN_HEIGHT = 3.0  // m
        const val FOLLOW_RANGE = 150.0     // m from where following started
        const val FOLLOW_MAX_ACC = 20.0    // m phone GPS accuracy needed
        const val FOLLOW_STALE_S = 3.0
        const val FOLLOW_MIN_BATTERY = 25
        const val POI_HYST = 2.0
        const val POLICY_CORRIDOR = 8.0    // the policy dodges sideways
        const val POLICY_MAX_SPEED = 4.0
        const val DEFAULT_GIMBAL = -10.0   // what the hazard/policy nets trained on
        const val M_PER_DEG_LAT = 111_195.0

        fun dist(a: P3, b: P3) = sqrt((a.e - b.e) * (a.e - b.e) + (a.n - b.n) * (a.n - b.n) + (a.u - b.u) * (a.u - b.u))
        fun along(a: P3, b: P3, p: P3): Double {
            val L = dist(a, b); if (L < 1e-6) return 0.0
            return ((p.e - a.e) * (b.e - a.e) + (p.n - a.n) * (b.n - a.n) + (p.u - a.u) * (b.u - a.u)) / L
        }
        fun wrap(d: Double): Double { var x = d % 360; if (x > 180) x -= 360; if (x < -180) x += 360; return x }
    }
}
