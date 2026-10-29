runoncepath(scriptpath():parent + "/geo").
runoncepath(scriptpath():parent + "/integrate").

function integrateAscent {
    parameter profile.
    parameter startVelocity is ship:velocity:orbit.
    parameter startMass is ship:mass.
    parameter startPosition is ship:position - body:position.
    parameter turnStartVelocity is 100.
    parameter turnAngle is 10.
    parameter dt is 1.
}

function integrateAscent {
    parameter profile.
    parameter startVelocity is ship:velocity:orbit.
    parameter startMass is ship:mass.
    parameter startPosition is ship:position - body:position.
    parameter turnStartVelocity is 100.
    parameter turnAngle is 10.
    parameter dt is 1.

    local startTime is time:seconds. // todo remove

    local mu is body:mu.
    local omega is body:angularvel.
    local bodyR is body:radius.

    local pos is startPosition.
    local vel is startVelocity.
    local a is pos:mag - bodyR.

    local m is startMass.
    local timeToHit is 0.

    local throttl is 0.

    function getAcc {
        parameter pos, vel, m is m.

        local a is pos:mag - bodyR.
        local surfVel is vel - vcrs(omega, pos).
        local aeroForce is profile:progradeAeroforceat(a, surfVel:mag) * surfVel:normalized.
        local acc is aeroForce / m.

        if throttl > 0 {
            set acc to acc + surfVel:normalized * profile:thrustat(a) * throttl / m.
        }
        set acc to acc - pos:normalized * mu / pos:sqrmagnitude.

        return acc.
    }

    until false {
        local a is pos:mag - bodyR.
        local surfVel is vel - vcrs(omega, pos).
        local surfVelNorm is surfVel:normalized.

        local aeroAcc is profile:progradeAeroforceat(a, surfVel:mag) * surfVelNorm / m.
        local gravityAcc is -pos:normalized * mu / pos:sqrmagnitude.

        local gParallel is vdot(getAcc, surfVelNorm).
        local gPerp is gravityAcc - gParallel.

        local targetForwardAcc is sqrt(30 * 30 - gPerp * gPerp).

        local requiredThrust is m * targetForwardAcc - m * gParallel + aeroAcc:mag.
        local maxThrust_ is profile:thrustat(a).

        set throttl to max(
            0, min(
                1,
                requiredThrust / maxThrust_
            )
        ).

        local acc is aeroAcc + gravityAcc + surfVelNorm * maxThrust_ * throttl / m.

        local updatedState is advanceRk4WithMass(pos, vel, m, acc, getAcc@, profile:massFlow() * throttl, dt).
        set pos to updatedState:pos.
        set vel to updatedState:vel.
        set m to updatedState:m.

        set a to pos:mag - bodyR.
        set timeToHit to timeToHit + dt.
    }

    local hitTime is startTime + timeToHit.

    return lex(
        "pos", pos,
        "vel", vel,
        "mass", m,
        "time", timeToHit,

        "timeToHit", hitTime - time:seconds,
        "hitTime", hitTime
    ).
}
