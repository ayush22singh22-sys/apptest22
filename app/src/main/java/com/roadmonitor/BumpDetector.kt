package com.roadmonitor

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.sqrt

/**
 * Tilt + bump/pothole detector. Pure Kotlin (no Android classes) so it can be tested on a laptop.
 *
 * Feed it raw accelerometer samples (m/s2, device axes). It
 *  1. estimates gravity with a ~1 s low-pass filter,
 *  2. gives tilt (pitch / roll) from that gravity vector,
 *  3. gives [vert]: the part of the remaining (dynamic) acceleration that points along gravity,
 *     i.e. the up/down jolt, no matter how the phone is mounted,
 *  4. turns a jolt bigger than [threshold] into ONE event once a short window has passed.
 *
 * Heuristic only: thresholds need tuning on real drives (use the Pothole / Speed breaker marker buttons
 * as ground truth).
 */
class BumpDetector(var threshold: Float = 3f, var minSpeedKmh: Float = 8f) {

    class Event(
        val tsMs: Long,        // time of the strongest jolt
        val peak: Float,       // strongest |vertical jolt| in m/s2
        val p2p: Float,        // max minus min vertical jolt inside the window
        val kind: String,      // "pothole" (drop first, then impact) or "bump" (rise first, e.g. speed breaker)
        val severity: String   // "low" | "medium" | "high"
    )

    var vert = 0f; private set
    var pitch = 0f; private set
    var roll = 0f; private set
    /** True only for the update() call in which a new event window opened (use it to snapshot GPS). */
    var justStarted = false; private set

    private companion object {
        const val TAU_S = 1.0f          // gravity filter time constant
        const val WARMUP_MS = 1500L     // let the gravity estimate settle
        const val WINDOW_MS = 350L      // how long one event is observed
        const val COOLDOWN_MS = 800L    // dead time after an event
        const val RAD2DEG = 57.29578f
    }

    private var ready = false
    private var gx = 0f
    private var gy = 0f
    private var gz = 0f
    private var firstTs = 0L
    private var lastTs = 0L
    private var lastEnd = Long.MIN_VALUE / 2
    private var inEvent = false
    private var evStart = 0L
    private var evMin = 0f
    private var evMax = 0f
    private var evMinT = 0L
    private var evMaxT = 0L

    /** @param speedKmh fresh GPS speed, or null when unknown. Returns a finished Event or null. */
    fun update(tsMs: Long, ax: Float, ay: Float, az: Float, speedKmh: Float?): Event? {
        justStarted = false
        if (!ready) {
            gx = ax; gy = ay; gz = az
            firstTs = tsMs; lastTs = tsMs
            ready = true
        } else {
            val dt = (tsMs - lastTs).coerceIn(1L, 200L) / 1000f
            val a = exp(-dt / TAU_S)
            gx = a * gx + (1f - a) * ax
            gy = a * gy + (1f - a) * ay
            gz = a * gz + (1f - a) * az
            lastTs = tsMs
        }
        val gm = sqrt(gx * gx + gy * gy + gz * gz)
        if (gm < 1f) return null   // free fall or bad data

        val ux = gx / gm
        val uy = gy / gm
        val uz = gz / gm
        vert = (ax - gx) * ux + (ay - gy) * uy + (az - gz) * uz
        pitch = atan2(-gx, sqrt(gy * gy + gz * gz)) * RAD2DEG
        roll = atan2(gy, gz) * RAD2DEG

        if (tsMs - firstTs < WARMUP_MS) return null

        if (inEvent) {
            if (vert < evMin) { evMin = vert; evMinT = tsMs }
            if (vert > evMax) { evMax = vert; evMaxT = tsMs }
            if (tsMs - evStart < WINDOW_MS) return null
            inEvent = false
            lastEnd = tsMs
            val peak = maxOf(abs(evMin), abs(evMax))
            val peakT = if (abs(evMin) >= abs(evMax)) evMinT else evMaxT
            val sev = when {
                peak >= threshold * 2.5f -> "high"
                peak >= threshold * 1.6f -> "medium"
                else -> "low"
            }
            return Event(peakT, peak, evMax - evMin, if (evMinT <= evMaxT) "pothole" else "bump", sev)
        }

        if (abs(vert) >= threshold && tsMs - lastEnd >= COOLDOWN_MS) {
            if (minSpeedKmh > 0f && (speedKmh == null || speedKmh < minSpeedKmh)) return null
            inEvent = true
            justStarted = true
            evStart = tsMs
            evMin = vert; evMax = vert
            evMinT = tsMs; evMaxT = tsMs
        }
        return null
    }
}
