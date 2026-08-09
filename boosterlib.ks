function integrateTrajectoryGemini {
    parameter simulationVelocity.
    parameter energyStep is 1000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.01.
    parameter dtMax is 0.5.

    local position is ship:position.
    local vel is simulationVelocity.
    local bodyPos is ship:body:position.
    local mu is ship:body:mu.
    local omega is ship:body:angularvel.
    local shipMass is ship:mass.
    local fwd is ship:facing:forevector.
    local bodyRadius is ship:body:radius.
    local startUT is time:seconds.

    local timeToHit is 0.
    lock altitude_ to (position - bodyPos):mag.

    until altitude_ < 5000 and altitude_ <= geoAtSimTime(position, startUT + timeToHit):terrainheight {
    //until altitude_ <= ship:body:geopositionof(position):terrainheight {

        // --- k1: evaluate at current state ---
        local s1 is getAccelAndEdot(position, vel, bodyPos, mu, omega, shipMass, ship:facing:forevector, bodyRadius).
        local acc1 is s1["acc"].
        local dEdt1 is s1["dEdt"].

        // --- adaptive step size from energy dissipation rate ---
        local dt is dtMax.
        if abs(dEdt1) > 0.0001 {
            set dt to abs(energyStep / dEdt1).
        }
        if dt < dtMin { set dt to dtMin. }
        if dt > dtMax { set dt to dtMax. }

        // --- RK2 midpoint step ---
        local posMid is position + vel * (dt / 2).
        local velMid is vel + acc1 * (dt / 2).

        local s2 is getAccelAndEdot(posMid, velMid, bodyPos, mu, omega, shipMass, ship:facing:forevector, bodyRadius).
        local acc2 is s2["acc"].

        set position to position + velMid * dt.
        set vel to vel + acc2 * dt.

        set timeToHit to timeToHit + dt.
    }

    local impactGeo is geoAtSimTime(position, startUT + timeToHit, ship:body:position - bodyPos).

    return lexicon(
        "impactPosition", position,
        "impactGeo", impactGeo,
        "timeToHit", timeToHit
    ).
}
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

function getAeroForce {
    parameter atAltitude.
    parameter atVelocity.
    parameter atAttitude.

    local currentAttitude is ship:facing.

    local atVelocityRelativeToBody is atAttitude:inverse * atVelocity.
    local atVelocityRelativeToCurrentAttitude is currentAttitude * atVelocityRelativeToBody.

    local forceRelativeToCurrentAttitude is addons:far:aeroforceat(atAltitude, atVelocityRelativeToCurrentAttitude).

    local forceRelativeToBody is currentAttitude:inverse * forceRelativeToCurrentAttitude.
    local force is atAttitude * forceRelativeToBody.

    return force.
}

function integrateTrajectoryOld {
   parameter simulationVelocity.
   parameter dt is 0.2.

   local position is ship:position.
   local vel is simulationVelocity.
   local altitude_ is ship:body:altitudeof(position).

   local timeToHit is 0.
   until altitude_ <= ship:body:geopositionof(position):terrainheight { //TODO i think it is better to use target height
        local positionFromBody is position - ship:body:position.
        local gAcc is -positionFromBody:normalized * (ship:body:mu / positionFromBody:sqrmagnitude).

        //local aeroForce is getAeroForce(altitude_, vel, lookdirup(-vel, ship:up:vector)).
        local selfVector is ship:position - ship:body:position.
        local vSurfaceCalc to vel - vcrs(ship:body:angularvel, selfVector).
        local aeroForceRaw is addons:far:aeroforceat(altitude_, ship:facing:forevector * vSurfaceCalc:mag).
        local aeroForce is -vSurfaceCalc:normalized * aeroForceRaw:mag.
        local aeroAcc is aeroForce / ship:mass.

        local acc is gAcc + aeroAcc.

        local accDt is acc * dt.

        set position to position + vel * dt + accDt * dt / 2.
        set vel to vel + acc * dt.
        set altitude_ to ship:body:altitudeof(position).

        set timeToHit to timeToHit + dt.
   }
   
   local rawGeo is ship:body:geopositionof(position).
   local rotationDegrees is (360 / ship:body:rotationperiod) * timeToHit.
   local impactGeo is latlng(rawGeo:lat, rawGeo:lng - rotationDegrees).

   return lexicon(
    "impactPosition", position,
    "impactGeo", impactGeo,
    "timeToHit", timeToHit
   ).
}

function displayPredictedHit {
    local hitData is integrateTrajectory(
        ship:velocity:orbit
    ).
    
    ADDONS:TR:SETTARGET(hitData["impactGeo"]).
}

// Computes total acceleration (gravity + zero-AoA drag) and the
// specific-energy dissipation rate at a given hypothetical state.
function getAccelAndEdot {
    parameter pos.
    parameter vel.
    parameter bodyPos.
    parameter mu.
    parameter omega.
    parameter shipMass.
    parameter fwd.
    parameter bodyRadius.

    local rVec is pos - bodyPos.
    local gAcc is -rVec:normalized * (mu / rVec:sqrmagnitude).

    local vSurf is vel - vcrs(omega, rVec).
    local speed is vSurf:mag.
    local altHere is rVec:mag - bodyRadius.
    set altHere to ship:body:altitudeof(pos).

    local aeroAcc is v(0,0,0).
    if speed > 0.1 {
        local aeroForceRaw is addons:far:aeroforceat(altHere, ship:facing:forevector * speed).
        local aeroForce is -vSurf:normalized * aeroForceRaw:mag.
        set aeroAcc to aeroForce / shipMass.
    }

    return lexicon(
        "acc", gAcc + aeroAcc,
        "dEdt", vdot(vel, aeroAcc),   // 0 in vacuum, negative under drag
        "altitude", altHere
    ).
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
    parameter energyStep is 2000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.01.
    parameter dtMax is 0.5.

    local hitTime is time:seconds.

    local bodyPos is ship:body:position.
    local omega is ship:body:angularvel.
    
    local position is ship:position.
    local vel is simulationVelocity.

    lock altitude_ to (position - bodyPos):mag - ship:body:radius.

    local prevPosition is position.
    local prevVel is vel.
    local prevAltitude is altitude_.
    local prevHitTime is hitTime.

    until altitude_ < 5000 and altitude_ <= geoAtSimTime(position, hitTime, ship:body:position - bodyPos):terrainheight {
    print altitude_.
        set prevPosition to position.
        set prevVel to vel.
        set prevAltitude to altitude_.
        set prevHitTime to hitTime.


        local positionFromBody is position - bodyPos.
        local gAcc is -positionFromBody:normalized * (ship:body:mu / positionFromBody:sqrmagnitude).
        
        local vSurf is vel - vcrs(omega, positionFromBody).
        local aeroForceRaw is addons:far:aeroforceat(altitude_, -ship:facing:forevector * vSurf:mag).
        local aeroForce is -vSurf:normalized * aeroforceRaw:mag.
        local aeroAcc to aeroForce / ship:mass.

        local acc is gAcc + aeroAcc.
        local dEdt is vdot(vel, aeroAcc).
        local dt is dtMax.
        if abs(dEdt) > 0.0001 {
            set dt to abs(energyStep / dEdt).
        }
        if dt < dtMin { set dt to dtMin. }
        if dt > dtMax { set dt to dtMax. }

        local accDt is acc * dt.
        set position to position + vel * dt + accDt * dt / 2.
        set vel to vel + acc * dt.

        set hitTime to hitTime + dt. 
    }

    local prevTerrainH is geoAtSimTime(prevPosition, prevHitTime, ship:body:position - bodyPos):terrainheight.
    local currTerrainH is geoAtSimTime(position, hitTime, ship:body:position - bodyPos):terrainheight.    
    local prevHeightAboveTerrain is prevAltitude - prevTerrainH.
    local curHeightAboveTerrain is altitude_ - currTerrainH.
    local frac is prevHeightAboveTerrain / (prevHeightAboveTerrain - curHeightAboveTerrain).

    set position to prevPosition + (position - prevPosition) * frac.
    set hitTime to prevHitTime + (hitTime - prevHitTime) * frac.

    local impactGeo is geoAtSimTime(position, hitTime, ship:body:position - bodyPos).
    printImpactOffset(impactGeo).

    print "" + addons:tr:timetillimpact + " - " + (hitTime - time:seconds).
    
    return lexicon(
        "impactPosition", position,
        "impactGeo", impactGeo,
        "timeToHit", hitTime - time:seconds
    ).
}

function geoAtSimTime {
    parameter pos.
    parameter targetUT.   // absolute universal time we actually want the geoposition for
    parameter bodyOffset is v(0, 0, 0).

    local rawGeo is ship:body:geopositionof(pos + bodyOffset).   // tainted by whatever "now" is at this exact call
    local nowUT is time:seconds.
    local degPerSec is 360 / ship:body:rotationperiod.
    local correctionDeg is degPerSec * (nowUT - targetUT).

    return latlng(rawGeo:lat, rawGeo:lng + correctionDeg).
}
