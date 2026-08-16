runoncepath("0:/den4ick240kos/boosterlib.ks").
parameter landingSite.
parameter pitchAngle to 0.           // degrees above horizontal for the commanded thrust vector; 0 = pure horizontal
parameter burning to false.          // start already burning (skips the heading-gate wait, e.g. when resuming a burn)
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.           // meters of frame-to-frame jitter to ignore
parameter minThrottle to 0.01.       // floor throttle whenever we're burning
parameter maxThrottle to 0.1.        // cap on how far we'll boost throttle to catch up
parameter headingGate to 3.5.        // degrees; cut throttle until ship faces within this of target bearing
parameter overshootLookahead to 0.3. // seconds; how far ahead we predict error to detect overshoot
parameter sensitivityFilter to 0.3.  // 0..1 lowpass strength on sensitivity; lower = smoother/slower, higher = snappier/noisier

set iterations to 0.
set bestError to 999999.
set havePrevErrAlongFacing to false.
set prevErrAlongFacing to 0.
set prevSampleTime to time:seconds.
set errRate to 0.           // "sensitivity": how fast the error along our facing direction is changing, per second
set filteredErrRate to 0.   // lowpassed sensitivity, used for overshoot prediction and throttle management

if burning {
    lock throttle to minThrottle.
}

until false {
    // Predict where we'd land if we cut thrust right now (current velocity).
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        landingSite:terrainheight
    ).
    local hitGeo is hitData["impactGeo"].

    // Track convergence: only count as improvement if it beats the noise floor,
    local hitDist is (hitGeo:position - landingSite:position):mag.
    if hitDist < bestError - noiseFloor {
        set bestError to hitDist.
    }  

    // Steer toward the target, with the commanded direction tilted by
    // pitchAngle degrees above horizontal. Burn once aligned (see heading
    // gate below); throttle level is managed further down.
    set horizErrVec to vectorExclude(ship:up:vector, landingSite:position - hitGeo:position).
    set errMag to horizErrVec:mag.

    // Heading gate: don't thrust until the ship is facing within headingGate
    // degrees of the actual commanded direction (including pitch tilt), so we
    // don't waste deltaV off-axis.
    local desiredDir is horizErrVec:normalized * cos(pitchAngle) + ship:up:vector:normalized * sin(pitchAngle).
    local headingErr is vectorangle(ship:facing:forevector, desiredDir).

    // Sensitivity: how fast the error changes specifically along the
    // direction we're actually facing/thrusting - that's the only axis our
    // burn can affect, so it's the number that actually predicts overshoot
    // and drives throttle management. We project the horizontal error
    // vector onto our current horizontal facing direction; as we close in
    // this scalar shrinks toward zero, and goes negative once we've flown
    // past the target along that axis.
    local facingHorizDir is vectorExclude(ship:up:vector, ship:facing:forevector):normalized.
    local errAlongFacing is vdot(horizErrVec, facingHorizDir).

    local nowTime is time:seconds.
    local dt is nowTime - prevSampleTime.
    if havePrevErrAlongFacing and dt > 0 {
        set errRate to (errAlongFacing - prevErrAlongFacing) / dt.
    } else {
        set errRate to 0.
    }
    set prevErrAlongFacing to errAlongFacing.
    set prevSampleTime to nowTime.
    set havePrevErrAlongFacing to true.

    // Exponential lowpass on the raw rate to reject frame-to-frame jitter
    // (trajectory-integration noise, physics tick variance) so a single
    // noisy sample can't flip the overshoot call or yank the throttle.
    set filteredErrRate to filteredErrRate + (errRate - filteredErrRate) * sensitivityFilter.

    // Linearly project the along-facing error forward by the lookahead
    // window. We're currently ahead of the target along our facing axis
    // (errAlongFacing > 0); if the projection crosses to zero or below
    // within that window, we're about to fly past it - overshoot.
    local predictedErrAlongFacing is errAlongFacing + filteredErrRate * overshootLookahead.
    local overshootPredicted is havePrevErrAlongFacing and (errAlongFacing > 0) and (predictedErrAlongFacing <= 0).

    print "head err " + headingErr at (0, 28).

    lock steering to lookdirup(desiredDir, ship:up:vector).

    // Ignition happens exactly once: as soon as we're pointed at the target
    // and still have meaningful error, we start burning.
    if not burning and errMag > 1 and headingErr <= headingGate {
        set burning to true.
    }

    // Throttle management: aim to null errAlongFacing within twice the
    // overshoot lookahead window. If our current (filtered) closing rate is
    // slower than what that target time needs, nudge throttle up above the
    // floor - capped at maxThrottle - so we don't dawdle into a long,
    // shallow approach. If we're already closing fast enough (or faster),
    // stay at the floor and let the overshoot predictor do its job.
    local desiredThrottle is minThrottle.
    if burning and errAlongFacing > 0 {
        local targetTime is 2 * overshootLookahead.
        local requiredClosingRate is errAlongFacing / targetTime.   // positive: speed needed to zero error in targetTime
        local currentClosingRate is -filteredErrRate.               // positive when actually closing
        if currentClosingRate < requiredClosingRate {
            local ratio is requiredClosingRate / max(currentClosingRate, 0.01).
            set desiredThrottle to minThrottle * ratio.
            if desiredThrottle > maxThrottle {
                set desiredThrottle to maxThrottle.
            }
        }
    }
    if burning {
        lock throttle to desiredThrottle.
    }

    print "err " + round(errMag, 1) + "m along " + round(errAlongFacing, 1) + "m sens " + round(filteredErrRate, 1) + "m/s thr " + round(desiredThrottle, 3) + " burn " + burning + " ovr " + overshootPredicted at (0, 22).
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
