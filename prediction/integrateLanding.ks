runoncepath(scriptpath():parent + "/integrate").
runoncepath(scriptpath():parent + "/geo").

global simulatedLandingBurnStartAltitude is 8500.

function integrateLanding {
    parameter profile.
    parameter targetAltitude is 0.
    parameter startVelocity is ship:velocity:orbit.
    parameter startMass is ship:mass.
    parameter startPosition is ship:position - body:position.
    parameter energyStep is 80000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.1.
    parameter dtMax is 8.

    parameter startTime is time:seconds. // todo remove 

    local geoAtSimTime is geoAtSimTimeProvider(). //todo remove

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
        local aeroForce is profile:retrogradeAeroforceat(a, -surfVel:mag) * -surfVel:normalized.
        local acc is aeroForce / m.

        if throttl > 0 {
            set acc to acc * 0.85 - surfVel:normalized * profile:thrustat(a) * throttl / m.
        }
        set acc to acc - pos:normalized * mu / pos:sqrmagnitude.

        return acc.
    }

    local shouldBreak is false.

    until a < targetAltitude - 400 or shouldBreak {
        local acc is getAcc(pos, vel).

        local surfVel is vel - vcrs(omega, pos).
        local dEdt is abs(vdot(surfVel, acc)).
        local dt is dtMax.
        if dEdt > 0.0001 {
            set dt to min(dtMax, max(dtMin, energyStep / dEdt)).
        }

        if throttl > 0 {
            local surfDeceleration is vdot(-surfVel:normalized, acc).
            if surfDeceleration * dt > surfVel:mag {
                set shouldBreak to true.
                set dt to surfVel:mag / surfDeceleration.
            }         
        } else if a > simulatedLandingBurnStartAltitude {
            local vertSpeed is vdot(vel, pos:normalized).
            if vertSpeed < 0 {
                local predictedA is a + vertSpeed * dt.
                if predictedA < simulatedLandingBurnStartAltitude {
                    set dt to max(dtMin, min(dt, (simulatedLandingBurnStartAltitude - a) * 1.01 / vertSpeed)). // overshot a little to start below the range
                }
            }
        }

        if throttl > 0 {
            local updatedState is advanceRk4WithMass(pos, vel, m, acc, getAcc@, profile:massFlow() * throttl, dt).
            set pos to updatedState:pos.
            set vel to updatedState:vel.
            set m to updatedState:m.
        } else {
            local updatedState is advanceRk4(pos, vel, acc, getAcc@, dt).
            set pos to updatedState:pos.
            set vel to updatedState:vel.
            if a <= simulatedLandingBurnStartAltitude {
                set throttl to 0.85.
            }
        }

        set a to pos:mag - bodyR.
        set timeToHit to timeToHit + dt.
    }

    local targetStoppingAltitude is targetAltitude + profile:heightFromOriginToBottom() + 15.

    set simulatedLandingBurnStartAltitude to simulatedLandingBurnStartAltitude - (a - targetStoppingAltitude) * 0.2.


    local hitTime is startTime + timeToHit.
    return lex(
        "pos", pos:normalized * (targetAltitude + bodyR),
        "mass", m,
        "time", timeToHit,
        "burnAlt", simulatedLandingBurnStartAltitude,

        "geo", geoAtSimTime(pos, hitTime),
        "timeToHit", hitTime - time:seconds,
        "hitTime", hitTime,
        "impactGeo", geoAtSimTime(pos, hitTime), // todo remove
        "burnAltitude", simulatedLandingBurnStartAltitude
    ).
}
