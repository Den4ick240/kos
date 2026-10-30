runoncepath("0:/den4ick240kos/boosterlib.ks").
runoncepath("0:/den4ick240kos/landingburn.ks").

function runApdg {
parameter landingSite.
parameter hitTime.

set steeringmanager:yawtorquefactor to 1.
set steeringmanager:pitchtorquefactor to 1.
set steeringmanager:yawts to 0.4.
set steeringmanager:pitchts to 0.4.
set steeringmanager:maxstoppingtime to 0.5.
SET STEERINGMANAGER:PITCHPID:KP TO STEERINGMANAGER:PITCHPID:KP * 2.0.
SET STEERINGMANAGER:YAWPID:KP TO STEERINGMANAGER:YAWPID:KP * 2.0.
SET STEERINGMANAGER:PITCHPID:Kd TO STEERINGMANAGER:PITCHPID:Kd / 2.0.
SET STEERINGMANAGER:YAWPID:Kd TO STEERINGMANAGER:YAWPID:Kd / 2.0.

local startPosition is ship:position - ship:body:position.
local g is ship:body:mu / startPosition:sqrmagnitude.
local touchdownThrust is availablethrust.
local terminalThrustAccel is touchdownThrust * 0.8 / mass.
local touchdownAcceleration is terminalThrustAccel - g.
local touchdownTime is 0.15.
local steeringLeadTime is 0.5.
local switchVelocity is touchdownTime * touchdownAcceleration.
local switchOffset is touchdownTime * switchVelocity / 2.

set hitTime to time:seconds + (hitTime - time:seconds) * 1.2 - touchdownTime.

local lastState is lexicon(
    "startTime", time:seconds,
    "startPosition", v(0, 0, 0),
    "startVelocity", v(0, 0, 0),
    "g", v(0, 0, 0),
    "timeToGo", 30,
    "a", v(0, 0, 0),
    "b", v(0, 0, 0),
    "c", v(0, 0, 0)
).

function getState {
    parameter startTime, startPosition, startVelocity, gravityVec, timeToGo.
    local af is -gravityVec:normalized * terminalThrustAccel.
    local vf is gravityVec:normalized * switchVelocity.
    local ttgsq is timeToGo * timeToGo.
    local ttgcb is ttgsq * timeToGo.
    local ttg4 is ttgsq * ttgsq.
    local gPlusAf is gravityVec + af.


    local a is af - 6 * (vf + startVelocity) / timeToGo - 12 * startPosition / ttgsq.
    local b is -6 * gPlusAf / timeToGo + (30 * vf + 18 * startVelocity) / ttgsq + 48 * startPosition / ttgcb.
    local c is 6 * gPlusAf / ttgsq - (24 * vf + 12 * startVelocity) / ttgcb - 36 * startPosition / ttg4.

    return lexicon(
        "startTime", startTime,
        "startPosition", startPosition,
        "startVelocity", startVelocity,
        "g", gravityVec,
        "timeToGo", timeToGo,
        "a", a,
        "b", b,
        "c", c
    ).
}

function getAcceleration {
    parameter state, atTime.
    return state["a"] + state["b"] * atTime + state["c"] * atTime * atTime.
}


local omega is ship:body:angularvel.

// Persistent arrow showing the target APDG thrust direction.
local apdgArrowLength is 100.
local apdgArrow is vecdraw(
    {return ship:position.},
    v(0, 0, 0),
    rgb(0, 1, 0),
    "apdg",
    1.0, true, 0.2, true, true
).

until ship:status = "LANDED" {
    local startPosition is ship:position - ship:body:position.
    local surfaceVelocity is ship:velocity:surface.
    local gAcc is -startPosition:normalized * ship:body:mu / startPosition:sqrmagnitude.

    local fictitiousAcc is (
        -2 * vcrs(omega, surfaceVelocity)
        - vcrs(omega, vcrs(omega, startPosition))
    ).

    local offsetLandingSitePosition is landingSite:altitudeposition(landingSite:terrainheight + getHeightFromOriginToBottom() + switchOffset). // todo offset by ship height
    local vsTargetPosition is ship:position - offsetLandingSitePosition.
    local height is vdot(up:vector, vsTargetPosition).
    //local ttg is 2 * vdot(up:vector, vsTargetPosition) / -verticalspeed.
    //local ttg is ship:velocity:surface:mag * 1.1 / (availablethrust / mass - gAcc:mag * vdot(up:vector, facing:vector)).
    //set ttg to max(0.1, ttg).
    //print ttg.
    local ttg is hitTime - time:seconds .
    print ttg at (0, 2).
    if ttg < 5 {
        set gear to true.
    }
    if ttg < touchdownTime or height < 20 {
        set gear to true.
        set apdgArrow:show to false.
        runLandingBurn(landingSite).
        break.
    }

    local state is getState(time:seconds, vsTargetPosition, surfaceVelocity, gAcc, ttg).

    
    local currentAcceleration is state["a"] - fictitiousAcc.
    local futureAcceleration is getAcceleration(state, steeringLeadTime) - fictitiousAcc.
    local apdgDir is futureAcceleration:normalized.
    set apdgArrow:vector to apdgDir * apdgArrowLength.
    set apdgArrow:label to "apdg " + round(futureAcceleration:mag, 1) + " m/s2".
    lock steering to lookdirup(futureAcceleration, ship:facing:topvector).
    lock throttle to max(0.1, min(1, 
        mass * currentAcceleration:mag / availablethrust
    )).
    wait 0.
}
}
