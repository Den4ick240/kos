runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.
parameter compensationGain to 0.1. // how much of each frame's downrange miss to integrate
parameter offsetDamping to 0.9.     // leaky-integrator damping (0..1): how much of old offset to keep
parameter maxOffset to 40000.       // clamp on virtual-target ground offset, meters
parameter hitAccuracy to 200.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.          // meters of frame-to-frame jitter to ignore
parameter minStagnantFrames to 8.   // consecutive non-improving frames required to call it converged

set requiredVelocity to -ship:velocity:orbit.
set predictedTimeOfFlight to 0.
set flightPathAngle to 45.

// Virtual target starts at the real landing site. Each frame it gets offset
// downrange by the measured drag-induced miss, so the (drag-free) flight path
// angle calc overshoots just enough to cancel drag.
set virtualTargetVector to landingSite:position - ship:body:position.
set predictedTargetVector to angleaxis(
    (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
    -ship:body:north:vector
) * virtualTargetVector.

lock requiredDeltaV to requiredVelocity - ship:velocity:orbit.
lock steering to lookdirup(requiredDeltaV, ship:up:vector).
lock throttle to choose min(1.0, requiredDeltaV:mag / 20) 
if vectorangle(ship:facing:vector, requiredDeltaV) < 5 else 0.

set iterations to 0.
set bestError to 999999.
set stagnantFrames to 0.
set downrangeOffset to 0.

until false {
    set predictedTimeOfFlight to getTimeOfFlight(flightPathAngle, predictedTargetVector).
    set predictedTargetVector to angleaxis(
        (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
        -ship:body:north:vector
    ) * virtualTargetVector.

    local lowAngle is max(2, flightPathAngle - 25).
    local highAngle is min(85, flightPathAngle + 25).

    from { local i is 0. } until i = 10 step { set i to i + 1. } do {
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

    // Predict where we'd land if we cut thrust right now (current velocity).
    // This is what the break condition is based on.
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        80000, 0.1, 12,
        "rk4", 0.1, 0.001
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

    // Predict where we'd land if we reach the target velocity, then offset the
    // virtual target strictly downrange so the drag-free flight path angle calc
    // compensates for drag shortening the reach.
    local aimData is integrateTrajectory(
        requiredVelocity,
        80000, 0.1, 8,
        "rk45", 0.1, 0.001
    ).
    local aimGeo is aimData["impactGeo"].

    // Downrange direction: horizontal component of the current surface velocity
    // (falls back to the horizontal ship->site direction if barely moving).
    local downrangeDir is v(0, 0, 0).
    if vectorExclude(ship:up:vector, ship:velocity:surface):mag > 0.1 {
        set downrangeDir to vectorExclude(ship:up:vector, ship:velocity:surface):normalized.
    } else {
        set downrangeDir to vectorExclude(ship:up:vector, landingSite:position - ship:position):normalized.
    }

    // Signed along-track miss of the aim prediction (+ = predicted hit overshoots
    // beyond the site, - = falls short). Only this downrange component is used.
    local alongErr is vdot(aimGeo:position - landingSite:position, downrangeDir).

    // Accumulate the downrange offset across frames with damping (leaky integrator)
    // instead of recomputing it fresh, so frame-to-frame noise can't make it jump.
    set downrangeOffset to offsetDamping * downrangeOffset - compensationGain * alongErr.
    if downrangeOffset > maxOffset { set downrangeOffset to maxOffset. }
    if downrangeOffset < -maxOffset { set downrangeOffset to -maxOffset. }

    set virtualTargetVector to (landingSite:position + downrangeOffset * downrangeDir) - ship:body:position.

    set iterations to iterations + 1.
    print "iter " + iterations + " hit " + round(hitDist, 1) + "m best " + round(bestError, 1) + "m stall " + stagnantFrames + "/" + minStagnantFrames + " along " + round(alongErr, 1) + "m off " + round(downrangeOffset, 1) + "m" at (0, 21).

    if hitDist <= hitAccuracy and stagnantFrames >= minStagnantFrames {
        unlock throttle.
        unlock steering.
        print "finished boostback".
        break.
    }

    wait 0.
}
