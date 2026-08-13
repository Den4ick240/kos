runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.
parameter compensationGain to 0.1. // how much of each frame's downrange miss to integrate
parameter offsetDamping to 0.9.     // leaky-integrator damping (0..1): how much of old offset to keep
parameter maxOffset to 40000.       // clamp on virtual-target downrange offset, meters
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.          // meters of frame-to-frame jitter to ignore
parameter minStagnantFrames to 3.   // consecutive non-improving frames to call it converged

local requiredVelocity to -ship:velocity:orbit.
local flightPathAngle to 45.
local downrangeOffset to 0.
local predictedTimeOfFlight to 0.

local predictedTargetVector to landingSite:position - ship:body:position.
local virtualTargetVector to predictedTargetVector.

local minThrottle is 0.

lock requiredDeltaV to requiredVelocity - ship:velocity:orbit.
lock steering to lookdirup(requiredDeltaV, ship:up:vector).
lock throttle to choose min(1.0, requiredDeltaV:mag / 20) 
if vectorangle(ship:facing:vector, requiredDeltaV) < 5 else minThrottle.

set bestError to 999999.
set stagnantFrames to 0.

until false {

    local lowAngle is max(2, flightPathAngle - 25).
    local highAngle is min(85, flightPathAngle + 25).

    from { local i is 0. } until i = 8 step { set i to i + 1. } do {
        local m1 is lowAngle + (highAngle - lowAngle) / 3.
        local m2 is highAngle - (highAngle - lowAngle) / 3.
        local dv1 is getRequiredDeltaVForFlightPathAngle(m1, virtualTargetVector).
        local dv2 is getRequiredDeltaVForFlightPathAngle(m2, virtualTargetVector).
        if dv1:mag < dv2:mag {
            set highAngle to m2.
        } else {
            set lowAngle to m1.
        }
    }
    set flightPathAngle to (lowAngle + highAngle) / 2.

    set requiredVelocity to getRequiredVelocityForFlightPathAngle(
        flightPathAngle,
        virtualTargetVector
    ).

    local aimData is integrateTrajectory(
        requiredVelocity,
        80000, 0.1, 12,
        "rk4", 0.1, 0.001
    ).
    set predictedTimeOfFlight to aimData["timeToHit"]

    local aimGeo is aimData["impactGeo"].
    local downrangeDir is vectorExclude(predictedTargetVector, predictedTargetVector - (ship:position - ship:body:position)):normalized.

    local alongErr is vdot(aimGeo:position - landingSite:position, downrangeDir).
    set downrangeOffset to offsetDamping * downrangeOffset - compensationGain * alongErr.
    if downrangeOffset > maxOffset { set downrangeOffset to maxOffset. }
    if downrangeOffset < -maxOffset { set downrangeOffset to -maxOffset. }

    set predictedTargetVector to angleaxis(
        (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
        -ship:body:north:vector
    ) * (landingSite:position - ship:body:position).    
    set virtualTargetVector is predictedTargetVector + downrangeDir * downrangeOffset.

    if requiredDeltaV:mag < 5 {
        unlock throttle.
        unlock steering.
        print "switching to error correction guidance".
        break.
    }

    if vectorangle(ship:facing:vector, requiredDeltaV) < 5 {
        set minThrottle to 0.1.
    }

    wait 0.
}

runpath("0:/den4ick240kos/boostback_phase2.ks", landingSite).
