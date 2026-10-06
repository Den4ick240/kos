runoncepath(scriptpath():parent + "/integrateLanding").

parameter compensationGain to 0.5. // how much of each frame's downrange miss to integrate
parameter offsetDamping to 0.98.     // leaky-integrator damping (0..1): how much of old offset to keep
parameter maxOffset to 40000.       // clamp on virtual-target downrange offset, meters

function getRequiredVelocityForFlightPathAngle {
    parameter flightPathAngle, targetVector.
    parameter currentVelocity is ship:velocity:orbit.
    parameter selfVector is ship:position - ship:body:position.

    local flightPathCos is cos(flightPathAngle).
    if abs(flightPathCos) < 0.0001 { return currentVelocity. }

    local flightAngle is vectorangle(selfVector, targetVector).
    local selfMagnitude is selfVector:mag.
    local targetMagnitude is targetVector:mag.

    local denomBracket is (
        selfMagnitude / targetMagnitude - cos(flightAngle + flightPathAngle) / flightPathCos
    ).

    if denomBracket <= 0 { return currentVelocity. }

    local desiredVelocitySquared is (
        ship:body:mu * (1 - cos(flightAngle))
    ) / (
        selfMagnitude * flightPathCos * flightPathCos * denomBracket 
    ).

    if desiredVelocitySquared <= 0 { return currentVelocity. }

    local desiredVelocityMagnitude is sqrt(desiredVelocitySquared ).

    local radialOut is selfVector / selfMagnitude.
    local desiredNormal is vectorCrossProduct(selfVector, targetVector):normalized.
    local desiredDirection is vectorCrossProduct(desiredNormal, radialOut):normalized.

    local desiredVelocityVector is desiredVelocityMagnitude * (
        desiredDirection * flightPathCos + radialOut * sin(flightPathAngle)
    ).

    return desiredVelocityVector.
}

local lowAngle is -85.
local highAngle is 85.
local flightPathAngle is 0.
local timeOfFlight is 0.
local downrangeOffset to 0.
local crossrangeOffset to 0.
local virtualTargetVector is v(0, 0, 0).

function initializeBoostbackStateInitialTarget {
    parameter targetVector.
    set virtualTargetVector to targetVector.
}

function optimizeBoostbackState {
    parameter profile, targetVector. 
    parameter vel is ship:velocity:orbit.
    parameter selfVector is ship:position - ship:body:position.
    parameter startMass is ship:mass.
    parameter _body is ship:body.

    set flightPathAngle to findBestAngle(lowAngle, highAngle, virtualTargetVector, vel, selfVector).
    set lowAngle to max(-89, flightPathAngle - 25). // update for the next iteration
    set highAngle to min(89, flightPathAngle + 25).

    local requiredVelocity is getRequiredVelocityForFlightPathAngle(flightPathAngle, virtualTargetVector, vel, selfVector).
    local requiredDeltaV is requiredVelocity - vel.
    local massAfterBurn is startMass / (
        constant:e ^ (
            requiredDeltaV:mag / (profile:ispat(selfVector:mag - ship:body:radius) * constant:g0)
        )
    ).

    local landing is integrateLanding(
        profile,
        targetVector:mag - _body:radius,
        requiredVelocity,
        massAfterBurn,
        selfVector
    ).
    set timeOfFlight to landing:time.
    set predictedTargetVector to angleaxis(
        _body:angularvel:mag * constant:radtodeg * timeOfFlight, 
        _body:angularvel
    ) * targetVector.

    local downrangeDir is vxcl(predictedTargetVector, selfVector):normalized.
    local crossrangeDir is vcrs(predictedTargetVector, downrangeDir):normalized.

    local err is landing:pos - predictedTargetVector.
    local downrangeErr is vdot(err, downrangeDir).
    local crossrangeErr is vdot(err, crossrangeDir).

    set downrangeOffset to offsetDamping * downrangeOffset - compensationGain * downrangeErr.
    if downrangeOffset > maxOffset { set downrangeOffset to maxOffset. }
    else if downrangeOffset < -maxOffset { set downrangeOffset to -maxOffset. }

    set crossrangeOffset to offsetDamping * crossrangeOffset - compensationGain * crossrangeErr.
    if crossrangeOffset > maxOffset { set crossrangeOffset to maxOffset. }
    else if crossrangeOffset < -maxOffset { set crossrangeOffset to -maxOffset. }

    set virtualTargetVector to predictedTargetVector 
            + (downrangeDir * downrangeOffset) 
            + (crossrangeDir * crossrangeOffset).


    return lex(
        "flightPathAngle", flightPathAngle,
        "requiredVelocity", requiredVelocity,
        "requiredDeltaV", requiredDeltaV,
        "massAfterBurn", massAfterBurn,
        "landing", landing
    ).
}

function findBestAngle {
    parameter lowAngle.
    parameter highAngle. 
    parameter targetVector.
    parameter currentVelocity.
    parameter selfVector.

    function getRequiredDeltaVForFlightPathAngle {
        parameter flightPathAngle.
        return getRequiredVelocityForFlightPathAngle(flightPathAngle, targetVector, currentVelocity, selfVector) - currentVelocity.
    }

    from { local i is 0. } until i = 8 step { set i to i + 1. } do {
        local m1 is lowAngle + (highAngle - lowAngle) / 3.
        local m2 is highAngle - (highAngle - lowAngle) / 3.
        local dv1 is getRequiredDeltaVForFlightPathAngle(m1).
        local dv2 is getRequiredDeltaVForFlightPathAngle(m2).
        if dv1:mag < dv2:mag {
            set highAngle to m2.
        } else {
            set lowAngle to m1.
        }
    }

    return (lowAngle + highAngle) / 2.
}
