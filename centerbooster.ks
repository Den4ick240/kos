
function setupCenterBoosterThrottling {

local coreStage is core:part:decoupledin.

local selectedEngines is list().

for eng in ship:engines {
    if eng:decoupledin = coreStage {
        selectedEngines:add(eng).
    }
}

if selectedEngines:lenght = ship:engines:length {
    return.
}

wait until selectedEngines[0]:thrust <> 0.

when true then {
    if ship:engines:length = selectedEngines:length {
        for eng in selectedEngines {
            set eng:thrustlimit to 100.
        }
        return false.
    }


    local selectedThrust is 0.
    for eng in selectedEngines {
        set selectedThrust to selectedThrust + eng:maxthrust.
    }

    local requiredThrust is 1.5 * ship:mass * constant:G0.
    local otherThrust is ship:maxthrust - selectedThrust.
    local allowedThrust to requiredThrust - otherThrust.

    local newThrottle is max(
        0.5,
        min(
            1,
            allowedThrust / selectedThrust
        )
    ).
    for eng in selectedEngines {
        set eng:thrustlimit to newThrottle * 100.
    }
    wait 0.
    return true.
}

}

setupCenterBoosterThrottling().

runpath(scriptpath():parent + "/booster.ks").
