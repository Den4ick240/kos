global simulatedLandingBurnStartAltitude is 8500.

function integrateLanding {
    parameter profile.
    parameter targetAltitude is 0.
    parameter startVelocity is ship:velocity:orbit.
    parameter startMass is ship:mass.
    parameter startPosition is ship:position - body:position.
    parameter startTime is time:seconds.
    parameter energyStep is 80000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.1.
    parameter dtMax is 8.


    local bodyPos is body:position.
    local northDir is (latlng(90, 0):position - bodyPos):normalized.
    local lon0Dir is (latlng(0, 0):position - bodyPos):normalized.
    local lon90Dir is (latlng(0, 90):position - bodyPos):normalized.
    local degPerSec is 360 / body:rotationperiod.
    local calculationStartTime is time:seconds.

    function geoAtSimTime {
        parameter pos, targetTime.
        local posMag is pos:mag.
        local lat_ is arcsin(vdot(pos, northDir) / posMag).
        local lon_ is arctan2(vdot(pos, lon90Dir), vdot(pos, lon0Dir)) - degPerSec * (targetTime - calculationStartTime).
        until lon_ <= 180 { set lon_ to lon_ - 360. }
        until lon_ >= -180 { set lon_ to lon_ + 360. }

        return latlng(lat_, lon_).
    }

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
    print round(simulatedLandingBurnStartAltitude) + " " + round(hitTime - time:seconds) at (0, 0).
    return lex(
        "pos", pos,
        "geo", geoAtSimTime(pos, hitTime),
        "timeToHit", hitTime - time:seconds,
        "hitTime", hitTime,
        "burnAlt", simulatedLandingBurnStartAltitude,

        "impactGeo", geoAtSimTime(pos, hitTime), // todo remove
        "burnAltitude", simulatedLandingBurnStartAltitude
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
