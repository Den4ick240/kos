runoncepath("0:/den4ick240kos/prediction/integrateLanding.ks").
runoncepath("0:/den4ick240kos/guidance.ks").
runoncepath("0:/den4ick240kos/surfaces.ks").
runoncepath("0:/den4ick240kos/apdg.ks").

function aerodescentGuidance {

parameter landingSite.

lock throttle to 0.

local desiredDir is srfretrograde:forevector.
local hitTime is 0.

local isAerodescent is true.

guidanceInit(landingSite).
surfaceFlipInit().

set steeringmanager:yawtorquefactor to 0.4.
set steeringmanager:pitchtorquefactor to 0.4.

lock steering to lookdirup(desiredDir, ship:facing:topvector).

when (ship:altitude < guidBurnAltitude) then {
    set isAerodescent to false.
    set ship:control:neutralize to True.
}

until not isAerodescent {
    local aa is guidanceUpdate().
    set desiredDir to aa["a"].
    set hitTime to aa["b"].
    if ship:altitude < guidBurnAltitude {
        break.
    }
    wait 0.
}

set steeringmanager:yawtorquefactor to 1.
set steeringmanager:pitchtorquefactor to 1.
runApdg(landingSite, hitTime).

}
