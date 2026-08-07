function selectLandingSite {
    until addons:tr:hasimpact { 
        print "Waiting for impact".
        wait 1. 
    }

    set impact to addons:tr:impactpos.

    set landingNames to list(
        //"KSC",
        "Island Airfield"
    ).

    set bestName to landingNames[0].
    set bestTarget to waypoint(bestName).
    set bestDistance to (impact:position - bestTarget:position):mag.


    FOR name IN landingNames {
        SET wp TO WAYPOINT(name).
        SET pos TO wp:GEOPOSITION.

        SET distance TO (impact:position - pos:position):mag.

        PRINT name + " distance: " + distance.

        IF distance < bestDistance {
            SET bestDistance TO distance.
            SET bestTarget TO pos.
            SET bestName TO name.
        }
    }

    PRINT "Selected landing site: " + bestName.

    ADDONS:TR:SETTARGET(bestTarget:geoposition).

    return bestTarget:geoposition.
}
