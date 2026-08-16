runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.
parameter compensationGain to 0.1. // how much of each frame's downrange miss to integrate
parameter offsetDamping to 0.9.     // leaky-integrator damping (0..1): how much of old offset to keep
parameter maxOffset to 40000.       // clamp on virtual-target downrange offset, meters
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.          // meters of frame-to-frame jitter to ignore
parameter minStagnantFrames to 3.   // consecutive non-improving frames to call it converged
parameter throttleKp to 0.0004.  // phase-2 PID proportional gain (per meter of error)
parameter throttleKi to 0.0. // phase-2 PID integral gain (drives out residual error)
parameter throttleKd to 0.04.   // phase-2 PID derivative gain (brakes the closing rate)
parameter maxThrottle to 0.1.   // phase-2 throttle cap so we don't slam the burn
parameter minThrottle to 0.01.   // phase-2 PID minimum throttle
parameter headingGate to 3.5.      // degrees; cut throttle until ship faces within this of target bearing

set requiredVelocity to -ship:velocity:orbit.
set predictedTimeOfFlight to 0.
set flightPathAngle to 45.
set downrangeOffset to 0.

set virtualTargetVector to landingSite:position - ship:body:position.
set predictedTargetVector to angleaxis(
    (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
    -ship:body:north:vector
) * virtualTargetVector.

lock requiredDeltaV to requiredVelocity - ship:velocity:orbit.
lock steering to lookdirup(requiredDeltaV, ship:up:vector).
lock throttle to choose min(1.0, requiredDeltaV:mag / 20) 
if vectorangle(ship:facing:vector, requiredDeltaV) < 5 else 0.

set phase to 1.
set iterations to 0.
set bestError to 999999.
set stagnantFrames to 0.
set hitPID to PIDLoop(throttleKp, throttleKi, throttleKd, minThrottle, maxThrottle).
set hitPID:setpoint to 0.

until false {
    // Phase 1: flight-path-angle guidance while the main boostback burn is ongoing.
    if phase = 1 {
        set predictedTimeOfFlight to getTimeOfFlight(flightPathAngle, predictedTargetVector).
        set predictedTargetVector to angleaxis(
            (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
            -ship:body:north:vector
        ) * virtualTargetVector.

        local lowAngle is max(2, flightPathAngle - 25).
        local highAngle is min(85, flightPathAngle + 25).

        from { local i is 0. } until i = 8 step { set i to i + 1. } do {
            local m1 is lowAngle + (highAngle - lowAngle) / 3.
            local m2 is highAngle - (highAngle - lowAngle) / 3.
            local dv1 is getRequiredDeltaVForFlightPathAngle(m1, predictedTargetVector).
            local dv2 is getRequiredDeltaVForFlightPathAngle(m2, predictedTargetVector).
            if dv1:mag < dv2:mag {
                set highAngle to m2.
            } else {
                set lowAngle to m1.
            }
        }
        set flightPathAngle to (lowAngle + highAngle) / 2.

        set requiredVelocity to getRequiredVelocityForFlightPathAngle(
            flightPathAngle,
            predictedTargetVector
        ).

        // Drag compensation for the old guidance: predict the hit at the target
        // velocity and offset the virtual target strictly downrange so the
        // drag-free flight path angle calc overshoots enough to cancel drag.
        local aimData is integrateTrajectory(
            requiredVelocity,
            80000, 0.1, 12,
            "rk4", 0.1, 0.001,
            landingSite:terrainheight
        ).
        local aimGeo is aimData["impactGeo"].

        local downrangeDir is v(0, 0, 0).
        if vectorExclude(ship:up:vector, ship:velocity:surface):mag > 0.1 {
            set downrangeDir to vectorExclude(ship:up:vector, ship:velocity:surface):normalized.
        } else {
            set downrangeDir to vectorExclude(ship:up:vector, landingSite:position - ship:position):normalized.
        }

        local alongErr is vdot(aimGeo:position - landingSite:position, downrangeDir).
        set downrangeOffset to offsetDamping * downrangeOffset - compensationGain * alongErr.
        if downrangeOffset > maxOffset { set downrangeOffset to maxOffset. }
        if downrangeOffset < -maxOffset { set downrangeOffset to -maxOffset. }
        set virtualTargetVector to (landingSite:position + downrangeOffset * downrangeDir) - ship:body:position.

        print "along " + round(alongErr, 1) + "m off " + round(downrangeOffset, 1) + "m" at (0, 22).

        if requiredDeltaV:mag < 15 {
            unlock throttle.
            unlock steering.
            set phase to 2.
            set bestError to 999999.
            set stagnantFrames to 0.
            print "switching to error correction guidance".
        }
    }

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
    if phase = 2 {
        set horizErrVec to vectorExclude(ship:up:vector, landingSite:position - hitGeo:position).
        set errMag to horizErrVec:mag.

        // Heading gate: don't thrust until the ship is facing within headingGate
        // degrees of the target bearing, so we don't waste deltaV sideways.
        set facingHoriz to vectorExclude(ship:up:vector, ship:facing:forevector).
        set targetBearing to vectorExclude(ship:up:vector, landingSite:position - ship:position).
        local headingErr is vectorangle(facingHoriz, horizErrVec).

        // kOS built-in PID loop: input is the horizontal error, output is the
        // throttle (already clamped to [minThrottle, maxThrottle] by the PID).
        set desiredThrottle to hitPID:update(time:seconds, errMag).
    print "head err " + headingErr at (0, 28).

        if errMag > 1 {
            lock steering to lookdirup(horizErrVec, ship:up:vector).
if headingErr <= headingGate  {
             lock throttle to desiredThrottle.
}
else {
            lock throttle to 0.
}
        } else {
            lock throttle to 0.
        }
        print "err " + round(errMag, 1) + "m hdg " + round(headingErr, 1) + " rate " + round(hitPID:changerate, 1) + "m/s thr " + round(desiredThrottle, 3) at (0, 22).
    }

    set iterations to iterations + 1.
    print "phase " + phase + " iter " + iterations + " hit " + round(hitDist, 1) + "m best " + round(bestError, 1) + "m stall " + stagnantFrames + "/" + minStagnantFrames + " dV " + round(requiredDeltaV:mag, 1) at (0, 21).

    if hitDist <= hitAccuracy and stagnantFrames >= minStagnantFrames {
        unlock throttle.
        unlock steering.
        print "finished boostback".
        break.
    }

    wait 0.
}
