package dji.sampleV5.aircraft.flightlab

import android.content.Context
import android.os.Handler
import android.os.Looper
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen

/**
 * One controller / mission runner / pilot link for the whole app, so the sandbox shell and the
 * native Flight Lab panel drive the same drone. Flying stops (hover, RC regains control) only when
 * no Flight Lab screen is visible any more, not when you switch between them.
 */
object Lab {
    lateinit var ctl: FlightLabController; private set
    lateinit var runner: MissionRunner; private set
    lateinit var pilot: PilotLink; private set
    @Volatile var droneConnected = false; private set
    @Volatile private var listened: Boolean? = null
    /** Connection changes, e.g. so the screen can report a drop. */
    var onConnection: (Boolean) -> Unit = {}
    private fun setConnected(v: Boolean) {
        if (v == droneConnected) return
        droneConnected = v
        ctl.event("drone_connection $v")
        main.post { onConnection(v) }
    }

    /** Toast-style messages from the controller/runner go to whichever screen is in front. */
    var onMessage: (String) -> Unit = {}

    @Volatile private var ready = false
    val isReady get() = ready
    private var visible = 0
    private val main = Handler(Looper.getMainLooper())

    private var started = false

    fun init(context: Context) {
        if (started) return
        started = true
        val app = context.applicationContext
        ctl = FlightLabController(app)
        ctl.onMessage = { m -> main.post { onMessage(m) } }
        runner = MissionRunner(ctl, app.filesDir)
        runner.onMessage = { m -> main.post { onMessage(m) } }
        pilot = PilotLink(ctl, runner)
        runner.pilot = pilot
        runner.onLanded = {
            // Only a flight that was actually recorded gets saved and uploaded (recording is opt-in).
            if (ctl.capturing) {
                ctl.stopCapture()
                ctl.uploadLastSession(java.text.SimpleDateFormat("yyyy-MM-dd", java.util.Locale.US).format(java.util.Date()))
            }
        }
        ctl.start()
        PhoneGps.start(app)
        // The listener is the source of truth. The 1 s poll only fills in a drone that was already connected
        // before we started (listen() reports changes only); a null/blank get() must never count as "dropped".
        FlightControllerKey.KeyConnection.create().listen(this) { v -> if (v != null) { listened = v; setConnected(v) } }
        main.post(object : Runnable {
            override fun run() {
                val g = try { FlightControllerKey.KeyConnection.create().get() } catch (_: Throwable) { null }
                if (g == true && listened != false) setConnected(true)
                main.postDelayed(this, 1000)
            }
        })
        ready = true   // only after everything above exists: the JS bridge reads these from other threads
    }

    fun screenStarted() { visible++ }

    /** Called from onStop. Activity B's onStart runs before A's onStop, so switching screens never hits zero. */
    fun screenStopped() {
        visible = (visible - 1).coerceAtLeast(0)
        if (visible == 0 && ready) {
            ctl.abortManeuver("app left foreground")
            runner.abort("app left foreground")
        }
    }
}
