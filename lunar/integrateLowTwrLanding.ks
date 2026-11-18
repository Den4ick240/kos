runoncepath(scriptpath():parent:parent + "/prediction/integrate").

function integrateLowTwrLanding {
    parameter p.
    parameter energyStep is 80000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.1.
    parameter dtMax is 8.
    local criticalDt is 0.01.

    local mu is p:body:mu.
    local omega is p:body:angularvel.
    local bodyR is p:body:radius.

    local pos is p:position.
    local vel is p:velocity.
    local a is pos:mag - bodyR.

    local m is p:mass.
    local timeToHit is 0.

    local targetAltitude is p:targetAltitude.
    local throttl is 0.9.

    local massflow is throttl * p:massflow.
    local thrust is throttl * p:thrust.
    local dryMass is p:dryMass.

    local flameOutReached is false.
    local zeroSurfaceVelocityReached is false.
    local tooLowReached is false.
    local shouldBreak is false. // save instructions by only checking shouldBreak in a loop condition

    function getAcc {
        parameter pos, vel, m.

        local surfVel is vel - vcrs(omega, pos).

        return -surfVel:normalized * thrust / m
            - pos:normalized * mu / pos:sqrmagnitude.
    }

    function getDt {
        parameter acc.
        
        local surfVel is vel - vcrs(omega, pos).

        local dEdt is abs(vdot(surfVel, acc)).
        local dt is dtMax.

        if dEdt > 0.0001 {
            set dt to min(dtMax, max(dtMin, energyStep / dEdt)).
        }

        local maxMassDt is (m - dryMass) / massflow.
        if maxMassDt < dt {
            set dt to max(criticalDt, maxMassDt).
            set shouldBreak to true.
            set flameOutReached to true.
        }

        local surfDecel is vdot(-surfVel:normalized, acc).
        local maxSurfVelDt is surfVel:mag / surfDecel.
        if maxSurfVelDt < dt {
            set dt to max(criticalDt, maxSurfVelDt).
            set shouldBreak to true.
            set zeroSurfaceVelocityReached to true.
        }

        return dt.
    }

    until shouldBreak {
        local acc is getAcc(pos, vel, m).

        local dt is getDt(acc).

        local updatedState is advanceRk4WithMass(pos, vel, m, acc, getAcc@, massflow, dt).
        set pos to updatedState:pos.
        set vel to updatedState:vel.
        set m to updatedState:m.

        set a to pos:mag - bodyR.
        set timeToHit to timeToHit + dt.

        if a < targetAltitude - 1000 {
            set tooLowReached to true.
            set shouldBreak to true.
        }
    }

    return lex(
        "position", pos,
        "mass", m,
        "time", timeToHit,
        "velocity", vel,
        "tooLow", tooLowReached,
        "flameOut", flameOutReached,
        "zeroSurfaceVelocity", zeroSurfaceVelocityReached
    ).
}
