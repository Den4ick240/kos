runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.
parameter compensationGain to 0.5. // how much of each frame's downrange miss to integrate
parameter offsetDamping to 0.98.     // leaky-integrator damping (0..1): how much of old offset to keep
parameter maxOffset to 40000.       // clamp on virtual-target downrange offset, meters
parameter hitAccuracy to 100.        // meters, break when predicted hit is this close
parameter noiseFloor to 5.          // meters of frame-to-frame jitter to ignore
parameter minStagnantFrames to 3.   // consecutive non-improving frames to call it converged

local requiredVelocity is -ship:velocity:orbit.
local flightPathAngle is 45.
local downrangeOffset is 0.
local crossrangeOffset is 0.
local predictedTimeOfFlight is 0.

local predictedTargetVector is landingSite:position - ship:body:position.
local virtualTargetVector is predictedTargetVector.

local minThrottle is 0.

lock requiredDeltaV to requiredVelocity - ship:velocity:orbit.
lock steering to lookdirup(requiredDeltaV, ship:up:vector).
// Calculate independent pitch and yaw deviations from the required burn vector.
// 90 minus the angle to the top/star vectors gives us the exact pitch/yaw deviation.
lock pitchErr to abs(90 - vang(ship:facing:topvector, requiredDeltaV)).
lock yawErr to abs(90 - vang(ship:facing:starvector, requiredDeltaV)).

// Calculate penalty factors from 0.0 (good) to 1.0 (bad).
// Yaw: 0 at 2 degrees, scales up to 1 at 5 degrees (range of 3).
lock yawPenalty to max(0, min(1, (yawErr - 3) / 6)).

// Pitch: 0 at 8 degrees, scales up to 1 at 12 degrees (range of 4).
lock pitchPenalty to max(0, min(1, (pitchErr - 6) / 6)).

// Take the worst of the two penalties
lock errorPenalty to max(yawPenalty, pitchPenalty).

// Smoothly interpolate the throttle
lock baseThrottle to min(1.0, requiredDeltaV:mag / 20).
lock throttle to minThrottle + (baseThrottle - minThrottle) * (1 - errorPenalty).
//lock throttle to choose min(1.0, requiredDeltaV:mag / 20) 
//if vectorangle(ship:facing:vector, requiredDeltaV) < 5 else minThrottle.

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
    set predictedTimeOfFlight to aimData["timeToHit"].
    local aimGeo is aimData["impactGeo"].

    set predictedTargetVector to angleaxis(
        (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
        -ship:body:north:vector
    ) * (landingSite:position - ship:body:position).    

    local desiredSurfaceVel is requiredVelocity.// - vcrs(ship:body:angularvel, ship:position - ship:body:position).
    local downrangeDir is v(0, 0, 0).
local crossrangeDir is v(0, 0, 0).
    if vectorExclude(ship:up:vector, desiredSurfaceVel):mag > 0.1 {
        set downrangeDir to vectorExclude(ship:up:vector, desiredSurfaceVel):normalized.
    set crossrangeDir to vcrs(ship:up:vector, downrangeDir):normalized.
    }  

    local alongErr is vdot(aimGeo:position - landingSite:position, downrangeDir).
local crossErr is vdot(aimGeo:position - landingSite:position, crossrangeDir).

    set downrangeOffset to offsetDamping * downrangeOffset - compensationGain * alongErr.
    if downrangeOffset > maxOffset { set downrangeOffset to maxOffset. }
    if downrangeOffset < -maxOffset { set downrangeOffset to -maxOffset. }

set crossrangeOffset to offsetDamping * crossrangeOffset - compensationGain * crossErr.

// Calculate lateral error

// Accumulate lateral offset
if crossrangeOffset > maxOffset { set crossrangeOffset to maxOffset. }
if crossrangeOffset < -maxOffset { set crossrangeOffset to -maxOffset. }

// Apply both dimensions to the target vector
set virtualTargetVector to predictedTargetVector 
                         + (downrangeDir * downrangeOffset) 
                         + (crossrangeDir * crossrangeOffset).

//    set virtualTargetVector to predictedTargetVector + downrangeDir * downrangeOffset.

    if requiredDeltaV:mag < 8 {
        print "switching to error correction guidance".
        break.
    }

    if vectorangle(ship:facing:vector, requiredDeltaV) < 5 {
        set minThrottle to 0.1.
    }

    wait 0.
}

runpath("0:/den4ick240kos/boostback/errProjWithThrottle.ks", landingSite, flightPathAngle, true).
