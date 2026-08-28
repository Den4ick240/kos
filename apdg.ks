runoncepath("0:/den4ick240kos/boosterlib.ks").
parameter landingSite.
parameter hitTime.




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
    local af is -gravityVec * 1.1.
    local vf is gravityVec:normalized * 40.
    local ttgsq is timeToGo * timeToGo.
    local ttgcb is ttgsq * timeToGo.
    local ttg4 is ttgsq * ttgsq.


    local a is af - 6 * (vf + startVelocity) / timeToGo - 12 * startPosition / ttgsq.
    local b is 0.6 * gravityVec / timeToGo + (30 * vf + 18 * startVelocity) / ttgsq + 48 * startPosition / ttgcb.
    local c is -0.6 * gravityVec / ttgsq - (24 * vf + 12 * startVelocity) / ttgcb - 36 * startPosition / ttg4.

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


until ship:status = "LANDED" {
    local startPosition is ship:position - ship:body:position.
    local gAcc is -startPosition:normalized * ship:body:mu / startPosition:sqrmagnitude.

    local offsetLandingSitePosition is landingSite:altitudeposition(landingSite:terrainheight + getHeightFromOriginToBottom() + 20). // todo offset by ship height
    local vsTargetPosition is ship:position - offsetLandingSitePosition.
    local height is vdot(up:vector, vsTargetPosition).
    //local ttg is 2 * vdot(up:vector, vsTargetPosition) / -verticalspeed.
    //local ttg is ship:velocity:surface:mag * 1.1 / (availablethrust / mass - gAcc:mag * vdot(up:vector, facing:vector)).
    //set ttg to max(0.1, ttg).
    //print ttg.
    local ttg is hitTime - time:seconds.
    if ttg < 5 {
        set gear to true.
    }
    if ttg < 1 or height < 20 {
        set gear to true.
        RUNPATH("1:/den4ick240kos/landingburn.ks").
        break.
    }

    local state is getState(time:seconds, vsTargetPosition, ship:velocity:surface, gAcc, ttg).

    
    local acceleration is state["a"].
    lock steering to lookdirup(acceleration, up:vector).
    lock throttle to max(0.1, min(1, 
        mass * acceleration:mag / availablethrust
    )).
    wait 0.
}
