runoncepath(scriptpath():parent:parent + "prediction/boostback").

parameter landingSite, boosterProfile.

function getStageMass {
    local fuel is stage:resourcelex("LiquidFuel").
    local fuelFraction is fuel:amount / fuel:capacity.
    return boosterProfile:dryMass + (boosterProfile:wetMass - boosterProfile:dryMass) * fuelFraction.
}

initializeBoostbackStateInitialTarget(landingSite:position - body:position).

local burnStart is time:seconds.
local massAfterBurn is optimizeBoostbackState(
    boosterProfile, 
    landingSite:position - body:position,
    ship:velocity:orbit,
    ship:position - ship:body:position,
    getStageMass()
):massAfterBurn. 

local nextUpdateTime is time:seconds + 5.
when nextUpdateTime < time:seconds then {
    local lastMassAfterBurn is massAfterBurn.
    local lastBurnStart is burnStart.

    local fuel is stage:resourcelex("LiquidFuel").
    local fuelFraction is fuel:amount / fuel:capacity.
    set burnStart to time:seconds.
    set massAfterBurn to optimizeBoostbackState(boosterProfile, 
        landingSite:position - body:position,
        ship:velocity:orbit,
        ship:position - ship:body:position,
        getStageMass()
    ):massAfterBurn. 

    local dMass is (massAfterBurn - lastMassAfterBurn) / (burnStart - lastBurnStart).

    if dMass >= 0 {
        return true
    }

    local massLeft is getStageMass() - boosterProfile:dryMass.
    local criticalTime is burnStart + massLeft / -dMass.

    print "separation in " + (criticalTime - time:seconds) at (0, 0).

    if criticalTime - time:seconds < 25 {
        stage.
        return false.
    } else {
        set nextUpdateTime to time:seconds + 5.
        return true.
    }
}
