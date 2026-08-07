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

function integrateTrajectory {
   parameter targetVector.
   parameter simulationVelocity.
   parameter dt is 0.5.

   local position is ship:position.
   local velocity is simulationVelocity.
   local altitude is ship:body:altitudeof(position).

   local timeToHit is 0.
   until altitude <= ship:geopositionof(position):terrainheight { //TODO i think it is better to use target height
        local positionFromBody is position - ship:body:position.
        local gAcc is -positionFromBody:normalized * (ship:body:mu / positionFromBody:sqrmagnitude).

        local aeroForce is getAeroForce(altitude, velocity, lookdirup(-velocity, ship:up:vector)).
        local aeroAcc is aeroForce / ship:mass.

        local acc is gAcc + aeroAcc.

        local accDt is acc * dt.

        set position to position + velocity * dt + accDt * dt / 2.
        set velocity to velocity + acc * dt.
        set altitude to ship:body:altitudeof(position).

        set timeToHit to timeToHit + dt.
   }
   
   local rawGeo is ship:geopositionof(position).
   local rotationDegrees is (360 / ship:body:rotationperiod) * timeToHit.
   local impactGeo is latlng(rawGeo:lat, rawGeo:lng - rotationDegrees).

   return lexicon(
    "impactPosition", position,
    "impactGeo", impactGeo,
    "timeToHit", timeToHit
   ).
}
