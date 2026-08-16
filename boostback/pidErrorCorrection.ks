runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.          // meters of frame-to-frame jitter to ignore
parameter minStagnantFrames to 3.   // consecutive non-improving frames to call it converged
parameter throttleKp to 0.0004.  // PID proportional gain (per meter of error)
parameter throttleKi to 0.0. // PID integral gain (drives out residual error)
parameter throttleKd to 0.04.   // PID derivative gain (brakes the closing rate)
parameter maxThrottle to 0.1.   // throttle cap so we don't slam the burn
parameter minThrottle to 0.01.   // PID minimum throttle
parameter headingGate to 3.5.      // degrees; cut throttle until ship faces within this of target bearing

set iterations to 0.
set bestError to 999999.
set stagnantFrames to 0.
set hitPID to PIDLoop(throttleKp, throttleKi, throttleKd, minThrottle, maxThrottle).
set hitPID:setpoint to 0.

until false {
    // Predict where we'd land if we cut thrust right now (current velocity).
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        80000, 0.1, 8,
        "rk4", 0.1, 0.001,
        landingSite:terrainheight
    ).
    local hitGeo is hitData["impactGeo"].
    addons:tr:settarget(hitGeo).

    // Track convergence: only count as improvement if it beats the noise floor,
    // otherwise count a stagnant frame so a few noisy frames can't fake a stall.
    local hitDist is (hitGeo:position - landingSite:position):mag.
    if hitDist < bestError - noiseFloor {
        set bestError to hitDist.
        set stagnantFrames to 0.
    } else {
        set stagnantFrames to stagnantFrames + 1.
    }

    // Phase 2: no flight path angle. Hold pitch at 0 (horizontal), steer yaw
    // toward the error, and once aligned within headingGate degrees, let the
    // built-in PID set the throttle so we don't overshoot the site.
    set horizErrVec to vectorExclude(ship:up:vector, landingSite:position - hitGeo:position).
    set errMag to horizErrVec:mag.

    // Heading gate: don't thrust until the ship is facing within headingGate
    // degrees of the target bearing, so we don't waste deltaV sideways.
    set facingHoriz to vectorExclude(ship:up:vector, ship:facing:forevector).
    local headingErr is vectorangle(facingHoriz, horizErrVec).

    // kOS built-in PID loop: input is the horizontal error, output is the
    // throttle (already clamped to [minThrottle, maxThrottle] by the PID).
    set desiredThrottle to hitPID:update(time:seconds, errMag).
    print "head err " + headingErr at (0, 28).

    if errMag > 1 {
        lock steering to lookdirup(horizErrVec, ship:up:vector).
        if headingErr <= headingGate {
            lock throttle to desiredThrottle.
        } else {
            lock throttle to 0.
        }
    } else {
        lock throttle to 0.
    }
    print "err " + round(errMag, 1) + "m hdg " + round(headingErr, 1) + " rate " + round(hitPID:changerate, 1) + "m/s thr " + round(desiredThrottle, 3) at (0, 22).

    set iterations to iterations + 1.
    print "phase 2 iter " + iterations + " hit " + round(hitDist, 1) + "m best " + round(bestError, 1) + "m stall " + stagnantFrames + "/" + minStagnantFrames at (0, 21).

    if hitDist <= hitAccuracy and stagnantFrames >= minStagnantFrames {
        unlock throttle.
        unlock steering.
        print "finished boostback".
        break.
    }

    wait 0.
}
