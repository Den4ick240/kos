runoncepath("0:/den4ick240kos/airbrakeForce.ks").
function getRequiredVelocityForFlightPathAngle {
    parameter flightPathAngle.
    parameter targetVector.

    local flightPathCos is cos(flightPathAngle).
    if abs(flightPathCos) < 0.0001 { return 999999. }

    local selfVector is ship:position - ship:body:position.
    local flightAngle is vectorangle(selfVector, targetVector).
    local selfMagnitude is selfVector:mag.
    local targetMagnitude is targetVector:mag.

    local denomBracket is (
        selfMagnitude / targetMagnitude - cos(flightAngle + flightPathAngle) / flightPathCos
    ).

    if denomBracket <= 0 { return 999999. }

    local desiredVelocitySquared is (
        ship:body:mu * (1 - cos(flightAngle))
    ) / (
        selfMagnitude * flightPathCos * flightPathCos * denomBracket 
    ).

    if desiredVelocitySquared <= 0 { return 999999. }

    local desiredVelocityMagnitude is sqrt(desiredVelocitySquared ).

    local radialOut is selfVector / selfMagnitude.
    local desiredNormal is vectorCrossProduct(selfVector, targetVector):normalized.
    local desiredDirection is vectorCrossProduct(desiredNormal, radialOut):normalized.

    local desiredVelocityVector is desiredVelocityMagnitude * (
        desiredDirection * flightPathCos + radialOut * sin(flightPathAngle)
    ).

    return desiredVelocityVector.
}

function getRequiredDeltaVForFlightPathAngle {
    parameter flightPathAngle.
    parameter targetVector.
    local requiredVelocity is getRequiredVelocityForFlightPathAngle(flightPathAngle, targetVector).
    return requiredVelocity - ship:velocity:orbit.
}

function getTimeOfFlight {
    PARAMETER flightPathAngle.
    PARAMETER targetVector.

    LOCAL flightPathCos IS COS(flightPathAngle).
    IF ABS(flightPathCos) < 0.0001 { RETURN 0. }

    LOCAL selfVector IS SHIP:POSITION - SHIP:BODY:POSITION.
    LOCAL flightAngle IS VECTORANGLE(selfVector, targetVector).
    LOCAL r0 IS selfVector:mag.
    LOCAL rt IS targetVector:mag.
    LOCAL mu IS SHIP:BODY:MU.

    // 1. Solve Hit Equation speed (vd)
    LOCAL denomBracket IS (r0 / rt) - (COS(flightAngle + flightPathAngle) / flightPathCos).
    IF denomBracket <= 0 { RETURN 0. }
    LOCAL vSq IS (mu * (1 - COS(flightAngle))) / (r0 * flightPathCos * flightPathCos * denomBracket).
    IF vSq <= 0 { RETURN 0. }
    LOCAL vd IS SQRT(vSq).

    // 2. Determine orbital shape parameters
    LOCAL p IS (r0 * r0 * vd * vd * flightPathCos * flightPathCos) / mu.
    LOCAL ecosTheta0 IS (p / r0) - 1.
    LOCAL esinTheta0 IS -(p / r0) * TAN(flightPathAngle).

    LOCAL e IS SQRT(ecosTheta0^2 + esinTheta0^2).
    IF e >= 1.0 { RETURN 0. } // Suborbital hit equations require elliptical orbits (e < 1)

    LOCAL theta0 IS ARCTAN2(esinTheta0, ecosTheta0). // Periapsis offset angle in degrees
    LOCAL a IS p / (1 - e^2).

    // 3. True anomalies at start and target (degrees)
    LOCAL thetaStart IS -theta0.
    LOCAL thetaTarget IS flightAngle - theta0.

    // 4. Eccentric Anomalies (radians)
    LOCAL sinE_start IS (SQRT(1 - e^2) * SIN(thetaStart)) / (1 + e * COS(thetaStart)).
    LOCAL cosE_start IS (e + COS(thetaStart)) / (1 + e * COS(thetaStart)).
    LOCAL E_start IS ARCTAN2(sinE_start, cosE_start) * CONSTANT:PI / 180.
FUNCTION evalHermiteSegment {
    PARAMETER x, x0, dx, y0, m0_out, y1, m1_in.
    LOCAL t IS (x - x0) / dx.
    LOCAL t2 IS t * t.
    LOCAL t3 IS t2 * t.
    RETURN (2*t3 - 3*t2 + 1)*y0 + (t3 - 2*t2 + t)*dx*m0_out + (-2*t3 + 3*t2)*y1 + (t3 - t2)*dx*m1_in.
}

FUNCTION getMachMultiplier {
    PARAMETER mach.
    IF mach <= 0.00 { RETURN 1.0000. }
    IF mach >= 5.00 { RETURN 3.0000. }

    IF mach < 0.85 {
        // Interval [0.00, 0.85] (dx = 0.85)
        RETURN evalHermiteSegment(mach, 0.00, 0.85, 1.0000, 0.00715953, 1.2500, 0.7780356).
    } ELSE IF mach < 1.10 {
        // Interval [0.85, 1.10] (dx = 0.25)
        RETURN evalHermiteSegment(mach, 0.85, 0.25, 1.2500, 0.7780356, 2.5000, 0.2492796).
    } ELSE {
        // Interval [1.10, 5.00] (dx = 3.90)
        RETURN evalHermiteSegment(mach, 1.10, 3.90, 2.5000, 0.2492796, 3.0000, 0.00).
    }
}

FUNCTION getAoAMultiplier {
    PARAMETER absTerm.
    LOCAL x IS MIN(ABS(absTerm), 1).
    IF x <= 0.00 { RETURN 0.0100. }
    IF x >= 1.00 { RETURN 2.4000. }

    IF x < 0.3420201 {
        // Interval [0.00, 0.3420201] (dx = 0.3420201)
        RETURN evalHermiteSegment(x, 0.00, 0.3420201, 0.0100, 0.00, 0.0600, 0.1750731).
    } ELSE IF x < 0.50 {
        // Interval [0.3420201, 0.50] (dx = 0.1579799)
        RETURN evalHermiteSegment(x, 0.3420201, 0.1579799, 0.0600, 0.1750731, 0.2400, 2.60928).
    } ELSE IF x < 0.7071068 {
        // Interval [0.50, 0.7071068] (dx = 0.2071068)
        RETURN evalHermiteSegment(x, 0.50, 0.2071068, 0.2400, 2.60928, 1.7000, 3.349777).
    } ELSE {
        // Interval [0.7071068, 1.00] (dx = 0.2928932)
        RETURN evalHermiteSegment(x, 0.7071068, 0.2928932, 1.7000, 3.349777, 2.4000, 0.00).
    }
}
    LOCAL sinE_target IS (SQRT(1 - e^2) * SIN(thetaTarget)) / (1 + e * COS(thetaTarget)).
    LOCAL cosE_target IS (e + COS(thetaTarget)) / (1 + e * COS(thetaTarget)).
    LOCAL E_target IS ARCTAN2(sinE_target, cosE_target) * CONSTANT:PI / 180.

    // 5. Mean Anomalies via Kepler's Equation (radians)
    LOCAL M_start IS E_start - e * SIN(E_start * CONSTANT:RADTODEG).
    LOCAL M_target IS E_target - e * SIN(E_target * CONSTANT:RADTODEG).

    LOCAL deltaM IS M_target - M_start.
    IF deltaM < 0 { SET deltaM TO deltaM + (2 * CONSTANT:PI). }

    // 6. Mean motion (rad/s) and final Time of Flight (seconds)
    LOCAL meanMotion IS SQRT(mu / (a^3)).
    RETURN deltaM / meanMotion.
}

function displayPredictedHit {
    local hitData is integrateTrajectory(
        ship:velocity:orbit
    ).
    
    ADDONS:TR:SETTARGET(hitData["impactGeo"]).
}

// Function to print difference in Lat, Lon, and Meters between Trajectories impactPos and targetPos
FUNCTION printImpactOffset {
    PARAMETER targetPos. // Expects a GeoCoordinates object, e.g., LATLNG(lat, lon) or TARGET:GEOPOSITION

    IF NOT ADDONS:TR:HASIMPACT {
        PRINT "Trajectories: No impact predicted." AT (0, 0).
        RETURN.
    }

    LOCAL impactPos IS ADDONS:TR:IMPACTPOS.

    // 1. Angular Differences (Latitude & Longitude in degrees)
    LOCAL dLat IS impactPos:LAT - targetPos:LAT.
    LOCAL dLng IS impactPos:LNG - targetPos:LNG.

    // Wrap longitude difference to [-180, 180] degrees
    UNTIL dLng <= 180 { SET dLng TO dLng - 360. }
    UNTIL dLng >= -180 { SET dLng TO dLng + 360. }

    // 2. Linear Differences in Meters
    // Position vector from target surface point to predicted impact point
    //LOCAL deltaVec IS impactPos:POSITION - targetPos:POSITION.

    //LOCAL totalDist IS deltaVec:MAG.
    //LOCAL distNorth IS VDOT(deltaVec, targetPos:NORTH:VECTOR).
    //LOCAL distEast IS VDOT(deltaVec, targetPos:EAST:VECTOR).

    // Print output
    PRINT "Delta Lat:   " + ROUND(dLat, 5) + " deg   " AT (0, 0).
    PRINT "Delta Lon:   " + ROUND(dLng, 5) + " deg   " AT (0, 1).
    //PRINT "Dist Total:  " + ROUND(totalDist, 2) + " m   " AT (0, 2).
    //PRINT "Dist North:  " + ROUND(distNorth, 2) + " m   " AT (0, 3).
    //PRINT "Dist East:   " + ROUND(distEast, 2) + " m   " AT (0, 4).
}


function integrateTrajectory {
    parameter simulationVelocity.
    parameter energyStep is 10000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.02.
    parameter dtMax is 4.


    local hitTime is time:seconds.

    local omega is ship:body:angularvel.
    
    local bodyPos is ship:body:position.
    local position is ship:position - ship:body:position.
    local vel is simulationVelocity.
    local aeroAcc is v(0,0,0).

    lock altitude_ to position:mag - ship:body:radius.

    local prevPosition is position.
    local prevVel is vel.
    local prevAltitude is altitude_.
    local prevHitTime is hitTime.

    function getForce {
        parameter position.
        parameter vel.
        parameter omega is omega.

        local gAcc is -position:normalized * (ship:body:mu / position:sqrmagnitude).
        local vSurf is vel - vcrs(omega, position).
        local aeroForceRaw is addons:far:aeroforceat(position:mag - ship:body:radius, -ship:facing:forevector * vSurf:mag).
        local aeroForce is -vSurf:normalized * aeroforceRaw:mag.
        set aeroAcc to (aeroForce ) / ship:mass.

        local acc is gAcc + aeroAcc.
        return lex(
            "full", acc,
            "aero", aeroAcc,
            "g", gAcc
        ).
    }

    until altitude_ < 5000 and altitude_ <= geoAtSimTime(position, hitTime):terrainheight {
        set prevPosition to position.
        set prevVel to vel.
        set prevAltitude to altitude_.
        set prevHitTime to hitTime.

        local r1 is getforce(position, vel).
        local acc1 is r1["full"].

        local dEdt is vdot(vel, r1["aero"]).
        local dt is dtMax.
        if abs(dEdt) > 0.0001 {
            set dt to abs(energyStep / dEdt).
        }
        if dt < dtMin { set dt to dtMin. }
        if dt > dtMax { set dt to dtMax. }

        local pos2 is position + vel * (dt / 2) + acc1 * (dt * dt / 8).
        local vel2 is vel + acc1 * (dt / 2).
        local acc2 is getForce(pos2, vel2)["full"].

        local pos3 is position + vel * (dt / 2) + acc1 * (dt * dt / 8).
        local vel3 is vel + acc2 * (dt / 2).
        local acc3 is getForce(pos3, vel3)["full"].

        local pos4 is position + vel * dt + acc2 * (dt * dt / 2).
        local vel4 is vel + acc3 * dt.
        local acc4 is getForce(pos4, vel4)["full"].

        set position to position + vel * dt + (acc1 + acc2 + acc3) * (dt * dt / 6).
        set vel to vel + (acc1 + acc2 * 2 + acc3 * 2 + acc4) * (dt / 6).

        set hitTime to hitTime + dt. 
    }

    local prevTerrainH is geoAtSimTime(prevPosition, prevHitTime):terrainheight.
    local currTerrainH is geoAtSimTime(position, hitTime):terrainheight.    
    local prevHeightAboveTerrain is prevAltitude - prevTerrainH.
    local curHeightAboveTerrain is altitude_ - currTerrainH.
    local frac is prevHeightAboveTerrain / (prevHeightAboveTerrain - curHeightAboveTerrain).
    set position to prevPosition + (position - prevPosition) * frac.
    set hitTime to prevHitTime + (hitTime - prevHitTime) * frac.

    local impactGeo is geoAtSimTime(position, hitTime).

    clearScreen.
    printImpactOffset(impactGeo).

    print "" + addons:tr:timetillimpact + " - " + (hitTime - time:seconds) + ": " + (addons:tr:timetillimpact - (hitTime - time:seconds)) at (0, 2).
    print (vel - vcrs(omega, position)):mag at (0, 3).
    print aeroAcc:mag at (0, 4).
    print "predicted acceleration " + getForce(ship:position - ship:body:position, ship:velocity:orbit, ship:body:angularvel)["full"]:mag at (0, 6).
    print "predicted acceleration gravity " + getForce(ship:position - ship:body:position, ship:velocity:orbit, ship:body:angularvel)["g"]:mag at (0, 7).
    print "predicted acceleration aero " + getForce(ship:position - ship:body:position, ship:velocity:orbit, ship:body:angularvel)["aero"]:mag at (0, 8).
    print "impact alt" + (prevTerrainH + (currTerrainH - prevTerrainH) * frac) at (0, 10).
    set anArrow to vecdraw(
    {return ship:position.},
    {return getForce(ship:position - ship:body:position, ship:velocity:orbit, ship:body:angularvel)["full"].},
    rgb(1,0,0),
    "hui",
    1.0,
    true,0.2,true,true
    ).
    set anArrow2 to vecdraw(
    {return ship:position.},
    {return getForce(ship:position - ship:body:position, ship:velocity:orbit, ship:body:angularvel)["aero"].},
    rgb(0,1,0),
    "",
    1.0,
    true,0.2,true,true
    ).
    
    set impactGeo to geoAtSimTime(position, hitTime).
    return lexicon(
        "impactPosition", position,
        "impactGeo", impactGeo,
        "timeToHit", hitTime - time:seconds
    ).
}


function geoAtSimTime {
    parameter pos.
    parameter targetUT.   // absolute universal time we actually want the geoposition for

    local nowUT is time:seconds.
    local rawGeo is ship:body:geopositionof(ship:body:position + pos).   // tainted by whatever "now" is at this exact call
    //local degPerSec is 360 / ship:body:rotationperiod.
    local degPerSec is ship:body:angularvel:mag * constant:RadToDeg.
    local correctionDeg is degPerSec * (nowUT - targetUT).

    return ship:body:geopositionlatlng(rawGeo:lat, rawGeo:lng + correctionDeg).
}
