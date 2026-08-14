runoncepath("0:/den4ick240kos/boosterlib.ks").
parameter landingSite.
parameter pitchAngle to 0.           // degrees above horizontal for the commanded thrust vector; 0 = pure horizontal
parameter burning to false.          // start already burning (skips the heading-gate wait, e.g. when resuming a burn)
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.           // meters of frame-to-frame jitter to ignore
parameter minThrottle to 0.01.       // fixed throttle used whenever we're burning
parameter headingGate to 3.5.        // degrees; cut throttle until ship faces within this of target bearing
parameter overshootLookahead to 0.3. // seconds; how far ahead we predict error to detect overshoot
parameter sensitivityFilter to 0.3.  // 0..1 lowpass strength on sensitivity; lower = smoother/slower, higher = snappier/noisier

set iterations to 0.
set bestError to 999999.
set havePrevErrVec to false.
set prevErrVec to V(0, 0, 0).
set prevSampleTime to time:seconds.
set errVecRate to V(0, 0, 0).  // "sensitivity": how fast the error vector itself is changing, per second
set filteredErrVecRate to V(0, 0, 0). // lowpassed sensitivity, used for the actual overshoot prediction

if burning {
    lock throttle to minThrottle.
}

until false {
    // Predict where we'd land if we cut thrust right now (current velocity).
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        80000, 0.1, 8,
        "rk4", 0.1, 0.001
    ).
    local hitGeo is hitData["impactGeo"].

    // Track convergence: only count as improvement if it beats the noise floor,
    local hitDist is (hitGeo:position - landingSite:position):mag.
    if hitDist < bestError - noiseFloor {
        set bestError to hitDist.
    }  

    // Steer toward the target, with the commanded direction tilted by
    // pitchAngle degrees above horizontal. Burn at a fixed low throttle
    // once aligned (see heading gate below).
    set horizErrVec to vectorExclude(ship:up:vector, landingSite:position - hitGeo:position).
    set errMag to horizErrVec:mag.

    // Heading gate: don't thrust until the ship is facing within headingGate
    // degrees of the actual commanded direction (including pitch tilt), so we
    // don't waste deltaV off-axis.
    local desiredDir is horizErrVec:normalized * cos(pitchAngle) + ship:up:vector:normalized * sin(pitchAngle).
    local headingErr is vectorangle(ship:facing:forevector, desiredDir).

    // Sensitivity: how much the error VECTOR itself changes per second, not
    // just its magnitude. This captures both closing speed and any curvature
    // in the approach (e.g. still slewing sideways as we close in).
    local nowTime is time:seconds.
    local dt is nowTime - prevSampleTime.
    if havePrevErrVec and dt > 0 {
        set errVecRate to (horizErrVec - prevErrVec) / dt.
    } else {
        set errVecRate to V(0, 0, 0).
    }
    set prevErrVec to horizErrVec.
    set prevSampleTime to nowTime.
    set havePrevErrVec to true.

    set filteredErrVecRate to filteredErrVecRate + (errVecRate - filteredErrVecRate) * sensitivityFilter.

    local predictedErrVec is horizErrVec + filteredErrVecRate * overshootLookahead.
    local overshootPredicted is havePrevErrVec and (vdot(horizErrVec, predictedErrVec) < 0).

    print "head err " + headingErr at (0, 28).

    lock steering to lookdirup(desiredDir, ship:up:vector).

    if not burning and errMag > 1 and headingErr <= headingGate {
        lock throttle to minThrottle.
        set burning to true.
    }

    print "err " + round(errMag, 1) + "m hdg " + round(headingErr, 1) + " sens " + round(filteredErrVecRate:mag, 1) + "m/s burn " + burning + " ovr " + overshootPredicted at (0, 22).
    set iterations to iterations + 1.
    print "phase 2 iter " + iterations + " hit " + round(hitDist, 1) + "m best " + round(bestError, 1) + "m" at (0, 21).

    if burning and (overshootPredicted and hitDist <= hitAccuracy) {
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
