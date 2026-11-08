package dji.sampleV5.aircraft.flightlab

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.Looper
import android.os.SystemClock

/** The phone's own GPS fix, for Follow me. Started once (needs location permission, which the app asks for). */
object PhoneGps : LocationListener {
    @Volatile var lat = Double.NaN; private set
    @Volatile var lon = Double.NaN; private set
    @Volatile var acc = Double.NaN; private set           // m, horizontal accuracy
    @Volatile var speed = Double.NaN; private set         // m/s over ground (NaN if unknown)
    @Volatile var bearing = Double.NaN; private set       // deg from north, valid when moving
    @Volatile var fixNs = 0L; private set                 // elapsedRealtimeNanos of the fix
    private var started = false

    fun ageS(): Double = if (fixNs == 0L) Double.POSITIVE_INFINITY else (SystemClock.elapsedRealtimeNanos() - fixNs) / 1e9

    @SuppressLint("MissingPermission")
    fun start(context: Context) {
        if (started) return
        val lm = context.getSystemService(Context.LOCATION_SERVICE) as? LocationManager ?: return
        try {
            lm.requestLocationUpdates(LocationManager.GPS_PROVIDER, 250L, 0f, this, Looper.getMainLooper())
            started = true
        } catch (_: SecurityException) { /* permission not granted yet: AppActivity calls start() again after it is */ }
        catch (_: Throwable) {}
    }

    override fun onLocationChanged(l: Location) {
        lat = l.latitude; lon = l.longitude
        acc = if (l.hasAccuracy()) l.accuracy.toDouble() else Double.NaN
        speed = if (l.hasSpeed()) l.speed.toDouble() else Double.NaN
        bearing = if (l.hasBearing()) l.bearing.toDouble() else Double.NaN
        fixNs = l.elapsedRealtimeNanos
    }

    @Deprecated("Deprecated in Java") override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}
    override fun onProviderEnabled(provider: String) {}
    override fun onProviderDisabled(provider: String) {}
}
