
function getStageProfile {
    parameter stageNumber.
    local fuels is uniqueset().
    local engs is list().
    local totalMassFlow is 0.
    local totalThrust is 0.
    for eng in ship:engines {
        if eng:stage >= stageNumber and eng:decoupledIn < stageNumber {
            engs:add(eng).
            for f in eng:consumedresources:values {
                fuels:add(f:name).
            }
            set totalMassFlow to totalMassFlow + eng:maxmassflow * eng:thrustLimit / 100.
            set totalThrust to totalThrust + eng:possiblethrust.
        }
    }

    local isp is 0.
    if totalMassFlow > 0 {
        set isp to totalThrust / (totalMassFlow * constant:g0).
    }

    local m is ship:mass.
    local dryMass is m.

    for part in ship:parts {
        if part:decoupledIn >= stageNumber or part:istype("Decoupler") and part:decoupledIn = (stageNumber - 1) {
            set m to m - part:mass.
            set dryMass to dryMass - part:mass.
        }  
        if part:decoupledIn = (stageNumber - 1) {
            for r in part:resources {
                if fuels:contains(r:name) {
                    set dryMass to dryMass - r:density * r:amount.
                }
            }
        }
    }

    return lex(
        "engines", engs,
        "massFlow", totalMassFlow,
        "thrust", totalThrust,
        "isp", isp,
        "mass", m,
        "dryMass", dryMass
    ).
}

function getStageProfileDeltaV {
    parameter p.
    return p:isp * constant:g0 * ln(p:mass / p:dryMass).
}
