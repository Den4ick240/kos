set offsetMeters to 5.

set file to "landing.json".

function saveLandingSite {
    local corePos is core:part:position - ship:position.
    local rightOffset is vdot(ship:facing:topvector, corePos).
    local topOffset is vdot(ship:facing:starvector, corePos).

    local landingSite is waypoint("Island Airfield"):geoposition.
    
    if abs(rightOffset) > 0.1 or abs(topOffset) > 0.1 {
        set landingSite to waypoint("KSC"):geoposition.

        local northVector is (
            body:geopositionlatlng(landingSite:lat + 0.001, landingSite:lng):position - landingSite:position
        ):normalized.

        local eastVector is (
            body:geopositionlatlng(landingSite:lat, landingSite:lng + 0.001):position - landingSite:position
        ):normalized.
        
        set landingSite to body:geopositionof(
            landingSite:position + northVector * topOffset * offsetMeters + eastVector * rightOffset * offsetMeters
        ).
    }

    writejson(landingSite, file).
}

function readLandingSite {
    local landingSite is readjson(file).
    print landingSite.
    return landingSite.
}
