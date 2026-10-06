runoncepath(scriptpath():parent + "/geo").

function integrateAscent {
    parameter profile.
    parameter startVelocity is ship:velocity:orbit.
    parameter startMass is ship:mass.
    parameter startPosition is ship:position - body:position.
    parameter dt is 1.

    local startTime is time:seconds. // todo remove

    local geoAtSimTime is geoAtSimTimeProvider().

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

        "geo", geoAtSimTime(pos, hitTime),
        "timeToHit", hitTime - time:seconds,
        "hitTime", hitTime
    ).
}

function advanceRk4 {
    parameter pos, vel, acc, getAcc, dt.

    local halfDt is dt / 2.
    local dtSqrdOver8 is dt * dt / 8.
    local dtOver6 is dt / 6.
    
    local pos2 is pos + vel * halfDt + acc * dtSqrdOver8.
    local vel2 is vel + acc * halfDt.
    local acc2 is getAcc(pos2, vel2).

    local pos3 is pos2.
    local vel3 is vel + acc2 * halfDt.
    local acc3 is getAcc(pos3, vel3).

    local pos4 is pos + vel * dt + acc3 * dt * halfDt.
    local vel4 is vel + acc3 * dt.
    local acc4 is getAcc(pos4, vel4).

    return lex(
        "pos", pos + vel * dt + (acc + acc2 + acc3) * dt * dtOver6,
        "vel", vel + (acc + acc2 * 2 + acc3 * 2 + acc4) * dtOver6
    ).
}

function advanceRk4WithMass {
    parameter pos, vel, m, acc, getAcc, massFlow, dt.

    local halfDt is dt / 2.
    local dtSqrdOver8 is dt * dt / 8.
    local dtOver6 is dt / 6.

    local mass2 is max(0.1, m - massFlow * halfDt).
    local pos2 is pos + vel * halfDt + acc * dtSqrdOver8.
    local vel2 is vel + acc * halfDt.
    local acc2 is getAcc(pos2, vel2, mass2).

    local mass3 is max(0.1, m - massFlow * halfDt).
    local pos3 is pos2.
    local vel3 is vel + acc2 * halfDt.
    local acc3 is getAcc(pos3, vel3, mass3).

    local mass4 is max(0.1, m - massFlow * dt).
    local pos4 is pos + vel * dt + acc3 * dt * halfDt.
    local vel4 is vel + acc3 * dt.
    local acc4 is getAcc(pos4, vel4, mass4).

    return lex(
        "pos", pos + vel * dt + (acc + acc2 + acc3) * dt * dtOver6,
        "vel", vel + (acc + acc2 * 2 + acc3 * 2 + acc4) * dtOver6,
        "m", max(0.1, m - massFlow * dt)
    ).
}
