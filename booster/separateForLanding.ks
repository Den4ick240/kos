runoncepath(scriptpath():parent:parent + "/prediction/boostback").

parameter landingSite, boosterProfile.

wait 1.

wait until ship:status <> "LANDED".
wait 70.

function getStageMass {
    local fuel is stage:resourceslex["LiquidFuel"].
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

local nextUpdateTime is time:seconds + 1.
local timeLeft is 50.
when nextUpdateTime < time:seconds then {
    local lastMassAfterBurn is massAfterBurn.
    local lastBurnStart is burnStart.

    local fuel is stage:resourceslex["LiquidFuel"].
    set burnStart to time:seconds.
    set massAfterBurn to optimizeBoostbackState(boosterProfile, 
        landingSite:position - body:position,
        ship:velocity:orbit,
        ship:position - ship:body:position,
        getStageMass()
    ):massAfterBurn. 

    local dMass is (massAfterBurn - lastMassAfterBurn) / (burnStart - lastBurnStart).

    if dMass >= 0 {
        return true.
    }

    local massLeft is getStageMass() - boosterProfile:dryMass.
    local criticalTime is burnStart + (massAfterBurn - boosterProfile:dryMass) / -dMass.
    set timeLeft to timeLeft * 0.9 + (criticalTime - time:seconds) * 0.1.


    print "separation in " + (criticalTime - time:seconds) at (0, 0).
    print "separation smo" + timeLeft at (0, 1).
    print "separation dma" + dMass at (0, 2).

    if timeLeft < 22 {
        lock throttle to 0.
        stage.
        return false.
    } else {
        set nextUpdateTime to time:seconds.
        return true.
    }
}
