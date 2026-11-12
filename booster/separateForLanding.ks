runoncepath(scriptpath():parent:parent + "/prediction/boostback").

parameter landingSite, boosterProfile.

print core:part:DECOUPLEDIN .
print STAGE:NUMBER.

print "awaiting nextstage".
WAIT UNTIL core:part:DECOUPLEDIN = (STAGE:NUMBER - 1).
print "awaiting launch".
wait until ship:status <> "PRELAUNCH".
print "awaiting altitude".
wait until ship:altitude > 2500.



local boosterFuel is 0.

for p in SHIP:PARTS {
    if p:DECOUPLER = CORE:PART:DECOUPLER {
        for r in p:RESOURCES {
            if r:NAME = "LiquidFuel" {
                set boosterFuel to r.
                break.
            }
        }
    }
    if boosterFuel <> 0 { break. }
}

wait 15.

//function getStageMass {
//    local fuel is stage:resourceslex["LiquidFuel"].
//    local fuelFraction is fuel:amount / fuel:capacity.
//    return boosterProfile:dryMass + (boosterProfile:wetMass - boosterProfile:dryMass) * fuelFraction.
//}
function getStageMass {
    local fuelFraction is boosterFuel:amount / boosterFuel:capacity.

    return boosterProfile:dryMass
        + (boosterProfile:wetMass - boosterProfile:dryMass) * fuelFraction.
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

local nextUpdateTime is time:seconds + 0.5.
local timeLeft is 50.
when nextUpdateTime < time:seconds then {
    local lastMassAfterBurn is massAfterBurn.
    local lastBurnStart is burnStart.

    local fuel is stage:resourceslex["LiquidFuel"].
    local computationStart is time:seconds.
    local boostbackState is optimizeBoostbackState(boosterProfile, 
        landingSite:position - body:position,
        ship:velocity:orbit,
        ship:position - ship:body:position,
        getStageMass()
    ).
    if boostbackState:requiredDeltaV:mag = 0 {
        return true.
    }
    set burnStart to computationStart.
    set massAfterBurn to boostbackState:massAfterBurn. 

    local dMass is (massAfterBurn - lastMassAfterBurn) / (burnStart - lastBurnStart).

    if dMass >= 0 {
        return true.
    }

    local massLeft is getStageMass() - boosterProfile:dryMass.
    local criticalTime is burnStart + (massAfterBurn - boosterProfile:dryMass - massLeft*0.01) / -dMass.
    set timeLeft to timeLeft * 0.8 + (criticalTime - time:seconds) * 0.2.


    print "separation in " + (criticalTime - time:seconds) at (0, 0).
    print "separation smo" + timeLeft at (0, 1).
    print "separation dma" + dMass at (0, 2).
    print "boostback dv " + boostbackState:requiredDeltaV at (0, 3).
    print "boostback m " + (getStageMass() - massAfterBurn) at (0, 4).

    if ship:modulesnamed("kOSProcessor"):length = 1 {
        return false.
    }

    if timeLeft < 13 {
        set warp to 0.
    }

    if timeLeft < 10 {
        lock throttle to 0.
        stage.
        return false.
    } else {
        set nextUpdateTime to time:seconds.
        return true.
    }
}
