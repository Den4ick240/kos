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

global burnMassFlowRate is 0.
when true then {
    list engines in _engList.
    local sum is 0.
    for eng in _engList {
        if eng:ignition {
            set sum to sum + eng:maxmassflow.
        }
    }
    set burnMassFlowRate to sum.
    print "bmfr " + ship:availablethrust() at (0, 5).
    return true.
}

function integrateTrajectory {
    parameter simulationVelocity.
    parameter targetAltitude is 0. // altitude above sea level (m) where the trajectory "hits"; used instead of terrain height
    parameter energyStep is 80000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.1.
    parameter dtMax is 8.

    local hitTime is time:seconds.
    local startUT is time:seconds.

    local body_ is ship:body.
    local omega is body_:angularvel.
    local bodyMu is body_:mu.
    local bodyRadius is body_:radius.
    local rotationPeriod is body_:rotationperiod.
    local degPerSec is 360 / rotationPeriod.
    local shipMass is ship:mass.
    
    local bodyPos is body_:position.
    local position is ship:position - bodyPos.

    local northDir is (latlng(90, 0):position - bodyPos):normalized.
    local lon0Dir is (latlng(0, 0):position - bodyPos):normalized.
    local lon90Dir is (latlng(0, 90):position - bodyPos):normalized.
    local vel is simulationVelocity.
    local aeroAcc is v(0,0,0).

    local burnAlt is 2500.
    local currentMass is shipMass.
    local inBurn is false.

    lock altitude_ to position:mag - bodyRadius.

    local prevPosition is position.
    local prevVel is vel.
    local prevAltitude is altitude_.
    local prevHitTime is hitTime.

    local steps is 0.

    function getForce {
        parameter position.
        parameter vel.
        parameter omega is omega.

        local gAcc is -position:normalized * (bodyMu / position:sqrmagnitude).
        local vSurf to vel - vcrs(omega, position).
        local aeroForceRaw is addons:far:aeroforceat(position:mag - bodyRadius, -ship:facing:forevector * vSurf:mag).
        local aeroForce is -vSurf:normalized * aeroforceRaw:mag.
        //local aeroForce is -vSurf:normalized * vdot(aeroforceRaw, -vSurf:normalized).
        //local aeroForce is lookdirup(-vSurf, position) * (ship:facing:inverse * aeroforceRaw).
        set aeroAcc to aeroForce / currentMass.

        local thrustAcc is v(0,0,0).
        local simAlt is position:mag - bodyRadius.
        if not inBurn and burnMassFlowRate > 0 and (simAlt - targetAltitude) < burnAlt {
            set inBurn to true.
        }
        if inBurn {
            print "vel " + vSurf:mag at (0, 6).
            print "alt " + simAlt at (0, 7).
            local pressure is 0.
            if body_:atm:exists {
                set pressure to body_:atm:altitudepressure(simAlt).
            }
            local curThrust is ship:availablethrustat(pressure).
            set thrustAcc to -vSurf:normalized * (curThrust / currentMass).
        }

        local acc is gAcc + aeroAcc + thrustAcc.
        return lex(
            "full", acc,
            "aero", aeroAcc,
            "g", gAcc
        ).
    }

    function geoAtSimTime {
        parameter pos.
        parameter targetUT.

        local posMag is pos:mag.
        local lat_ is arcsin(vdot(pos, northDir) / posMag).
        local lon_ is arctan2(vdot(pos, lon90Dir), vdot(pos, lon0Dir)) - degPerSec * (targetUT - startUT).

        until lon_ <= 180 { set lon_ to lon_ - 360. }
        until lon_ >= -180 { set lon_ to lon_ + 360. }

        return latlng(lat_, lon_).
    }

    until altitude_ <= targetAltitude {
        set prevPosition to position.
        set prevVel to vel.
        set prevAltitude to altitude_.
        set prevHitTime to hitTime.

        local r1 is getforce(position, vel).
        local acc1 is r1["full"].

        local dEdt is vdot(vel, r1["full"]).
        local dt is dtMax.
        if abs(dEdt) > 0.0001 {
            set dt to abs(energyStep / dEdt).
        }
        if dt < dtMin { set dt to dtMin. }
        if dt > dtMax { set dt to dtMax. }

        if not inBurn {
            local simBurnAlt is burnAlt + targetAltitude.
            if altitude_ > simBurnAlt {
                local vertSpeed is vdot(vel, position:normalized).
                if vertSpeed < 0 {
                    local projectedAlt is altitude_ + vertSpeed * dt.
                    if projectedAlt < simBurnAlt {
                        set dt to max(dtMin, min(dt, (simBurnAlt - altitude_) / vertSpeed)).
                    }
                }
            }
        }

        if acc1:mag * dt > vel:mag {
            break.
        }

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
        set steps to steps + 1.

        if not inBurn and altitude_ <= burnAlt + targetAltitude {
            set inBurn to true.
        }

        if inBurn {
            set currentMass to currentMass - burnMassFlowRate * dt.
            if currentMass < 0.1 { set currentMass to 0.1. }
            local vSurf is vel - vcrs(omega, position).
            local vertSpeed is vdot(vSurf, position:normalized).
            if vSurf:mag < 10 or vertSpeed > -5 or acc1:mag * dt > vSurf:mag {
                break.
            }
        }
    }

    if altitude_ > targetAltitude {
        set position to position:normalized * (targetAltitude + bodyRadius).
    } else {
        local prevHeightAboveTarget is prevAltitude - targetAltitude.
        local curHeightAboveTarget is altitude_ - targetAltitude.
        local frac is prevHeightAboveTarget / (prevHeightAboveTarget - curHeightAboveTarget).
        set position to prevPosition + (position - prevPosition) * frac.
        set hitTime to prevHitTime + (hitTime - prevHitTime) * frac.
    }
    print "Time to hit " + (hitTime - time:seconds) at (0, 20).

    local impactGeo is geoAtSimTime(position, hitTime).
    return lexicon(
        "impactPosition", position,
        "impactGeo", impactGeo,
        "timeToHit", hitTime - time:seconds,
        "simTimeToHit", hitTime - startUT,
        "steps", steps
    ).
}
