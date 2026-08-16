runoncepath("0:/den4ick240kos/boosterlib.ks").
parameter landingSite.
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.           // meters of frame-to-frame jitter to ignore
parameter minStagnantFrames to 3.    // consecutive non-improving frames to call it converged
parameter minThrottle to 0.01.       // fixed throttle used whenever we're burning
parameter headingGate to 3.5.        // degrees; cut throttle until ship faces within this of target bearing
parameter overshootLookahead to 0.1. // seconds; how far ahead we predict error to detect overshoot

set iterations to 0.
set bestError to 999999.
set stagnantFrames to 0.
set prevErrMag to -1.        // -1 means "no previous sample yet"
set prevSampleTime to time:seconds.
set errRate to 0.
set burning to false.        // once true, throttle is locked and never re-touched until stop

until false {
    // Predict where we'd land if we cut thrust right now (current velocity).
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
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
    // toward the error, and burn at a fixed low throttle once aligned.
    set horizErrVec to vectorExclude(ship:up:vector, landingSite:position - hitGeo:position).
    set errMag to horizErrVec:mag.

    // Heading gate: don't thrust until the ship is facing within headingGate
    // degrees of the target bearing, so we don't waste deltaV sideways.
    set facingHoriz to vectorExclude(ship:up:vector, ship:facing:forevector).
    local headingErr is vectorangle(facingHoriz, horizErrVec).

    // Frame-to-frame error rate, used to predict overshoot instead of a PID loop.
    local nowTime is time:seconds.
    local dt is nowTime - prevSampleTime.
    if prevErrMag >= 0 and dt > 0 {
        set errRate to (errMag - prevErrMag) / dt.
    } else {
        set errRate to 0.
    }
    set prevErrMag to errMag.
    set prevSampleTime to nowTime.

    // If error is closing and we predict it'll cross zero within the lookahead
    // window, we'd overshoot the site before the next real correction — cut now.
    local predictedErr is errMag + errRate * overshootLookahead.
    local overshootPredicted is (errRate < 0) and (predictedErr <= 0).

    print "head err " + headingErr at (0, 28).

    // Steering updates every frame regardless of burn state, so we keep
    // tracking the target as it drifts.
    lock steering to lookdirup(horizErrVec, ship:up:vector).

    // Ignition happens exactly once: as soon as we're pointed at the target
    // and still have meaningful error, we light the engine at minThrottle
    // and never touch the throttle lock again until the stop condition.
    if not burning and errMag > 1 and headingErr <= headingGate {
        lock throttle to minThrottle.
        set burning to true.
    }

    print "err " + round(errMag, 1) + "m hdg " + round(headingErr, 1) + " rate " + round(errRate, 1) + "m/s pred " + round(predictedErr, 1) + "m burn " + burning at (0, 22).
    set iterations to iterations + 1.
    print "phase 2 iter " + iterations + " hit " + round(hitDist, 1) + "m best " + round(bestError, 1) + "m stall " + stagnantFrames + "/" + minStagnantFrames at (0, 21).

    // Stop condition only matters once we're actually burning; before ignition
    // there's nothing to cut and nothing to overshoot.
    if burning and (overshootPredicted or (hitDist <= hitAccuracy and stagnantFrames >= minStagnantFrames)) {
        unlock throttle.
        unlock steering.
        if overshootPredicted {
            print "finished boostback (overshoot predicted)".
        } else {
            print "finished boostback".
        }
        break.
    }
    wait 0.
}
