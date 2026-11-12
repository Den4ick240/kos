runoncepath(scriptpath():parent:parent + "/prediction/integrateLanding").
runoncepath(scriptpath():parent:parent + "/prediction/getHitError").

local downrangePid is pidloop(0.5, 0, 0.1 -90, 90).
local crossrangePid is pidloop(0.1, 0, 0.1 -90, 90).

local filteredErr is vec(0, 0, 0).

function getAerodescentGuidance {
    parameter landingSite.
    local selfVector is ship:position - ship:body:position.
    local targetVector is landingSite - ship:body:position.

    local landing is integrateLanding(landingSite:terrainHeight).

    local planeNormal is vcrs(selfVector, targetVector).

    if planeNormal:sqrmag < 0.0001 {
        set planeNormal to vcrs(selfVector, ship:srfretrograde:forevector):normalized.
    } else {
        set planeNormal to planeNormal:normalized.
    }

    local crossAngleRad is arcsin(clamp(vdot(landing:pos, planeNormal)))
    

    local err is getHitError(selfVector, targetVector, landing).

    local maxAoa is min(50 / max(ship:q * constant:AtmToKPa, 0.05), 70).


}
