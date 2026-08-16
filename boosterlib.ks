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
    parameter method is "rk45".
    parameter rk45Rtol is 0.1.
    parameter rk45Atol is 0.001.

    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        80000, 0.1, 8,
        method, rk45Rtol, rk45Atol
    ).
    
    ADDONS:TR:SETTARGET(hitData["impactGeo"]).
}

function integrateTrajectory {
    parameter simulationVelocity.
    parameter energyStep is 80000.  // target |ΔE| per step, (m/s)^2 - tune this
    parameter dtMin is 0.1.
    parameter dtMax is 8.
    parameter method is "rk45".    // "rk45" (Dormand-Prince 5(4), adaptive) or "rk4"
    parameter rk45Rtol is 0.1.   // relative tolerance for rk45 error control
    parameter rk45Atol is 0.001.   // absolute tolerance for rk45 error control
    parameter targetAltitude is 0. // altitude above sea level (m) where the trajectory "hits"; used instead of terrain height

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

    lock altitude_ to position:mag - bodyRadius.

    local prevPosition is position.
    local prevVel is vel.
    local prevAltitude is altitude_.
    local prevHitTime is hitTime.

    local forceEvals is 0.
    local steps is 0.
    local rejectedSteps is 0.

    function getForce {
        parameter position.
        parameter vel.
        parameter omega is omega.

        set forceEvals to forceEvals + 1.

        local gAcc is -position:normalized * (bodyMu / position:sqrmagnitude).
        local vSurf is vel - vcrs(omega, position).
        local aeroForceRaw is addons:far:aeroforceat(position:mag - bodyRadius, -ship:facing:forevector * vSurf:mag).
        local aeroForce is -vSurf:normalized * aeroforceRaw:mag.
        //local aeroForce is -vSurf:normalized * vdot(aeroforceRaw, -vSurf:normalized).
        //local aeroForce is lookdirup(-vSurf, position) * (ship:facing:inverse * aeroforceRaw).
        set aeroAcc to (aeroForce ) / shipMass.

        local acc is gAcc + aeroAcc.
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

    if method = "rk45" {
        // Dormand-Prince 5(4) with embedded error estimate and FSAL reuse.
        // Cost: 7 force evals on the very first step, then 6 per accepted step
        // (plus 6 more per rejected try). Reuses getForce / geoAtSimTime above.
        local first is true.
        local prevK7a is v(0,0,0).
        local h is dtMax.
        local accept is false.
        local err is 0.
        local k7a is v(0,0,0).

        until altitude_ <= targetAltitude {
            set prevPosition to position.
            set prevVel to vel.
            set prevAltitude to altitude_.
            set prevHitTime to hitTime.

            // FSAL: first stage of this step = last stage of previous step
            local k1p is vel.
            local k1a is v(0,0,0).
            if first {
                local r1 is getForce(position, vel).
                set k1a to r1["full"].
                // seed step size from the same energy heuristic as rk4
                local dEdt is vdot(vel, r1["aero"]).
                set h to dtMax.
                if abs(dEdt) > 0.0001 { set h to abs(energyStep / dEdt). }
                if h < dtMin { set h to dtMin. }
                if h > dtMax { set h to dtMax. }
                set first to false.
            } else {
                set k1a to prevK7a.
            }

            set accept to false.
            local pos7 is v(0,0,0).
            local vel7 is v(0,0,0).
            until accept {
                local pos2 is position + k1p * (h / 5).
                local vel2 is vel + k1a * (h / 5).
                local k2a is getForce(pos2, vel2)["full"].
                local k2p is vel2.

                local pos3 is position + (k1p * (3/40) + k2p * (9/40)) * h.
                local vel3 is vel + (k1a * (3/40) + k2a * (9/40)) * h.
                local k3a is getForce(pos3, vel3)["full"].
                local k3p is vel3.

                local pos4 is position + (k1p * (44/45) - k2p * (56/15) + k3p * (32/9)) * h.
                local vel4 is vel + (k1a * (44/45) - k2a * (56/15) + k3a * (32/9)) * h.
                local k4a is getForce(pos4, vel4)["full"].
                local k4p is vel4.

                local pos5 is position + (k1p * (19372/6561) - k2p * (25360/2187) + k3p * (64448/6561) - k4p * (212/729)) * h.
                local vel5 is vel + (k1a * (19372/6561) - k2a * (25360/2187) + k3a * (64448/6561) - k4a * (212/729)) * h.
                local k5a is getForce(pos5, vel5)["full"].
                local k5p is vel5.

                local pos6 is position + (k1p * (9017/3168) - k2p * (355/33) + k3p * (46732/5247) + k4p * (49/176) - k5p * (5103/18656)) * h.
                local vel6 is vel + (k1a * (9017/3168) - k2a * (355/33) + k3a * (46732/5247) + k4a * (49/176) - k5a * (5103/18656)) * h.
                local k6a is getForce(pos6, vel6)["full"].
                local k6p is vel6.

                // 5th order solution = stage 7 (FSAL: k7 = f(pos7, vel7))
                set pos7 to position + (k1p * (35/384) + k3p * (500/1113) + k4p * (125/192) - k5p * (2187/6784) + k6p * (11/84)) * h.
                set vel7 to vel + (k1a * (35/384) + k3a * (500/1113) + k4a * (125/192) - k5a * (2187/6784) + k6a * (11/84)) * h.
                set k7a to getForce(pos7, vel7)["full"].

                // embedded error estimate e = b5 - b4 (k7p = vel7)
                local dPos is h * (k1p * (71/57600) - k3p * (71/16695) + k4p * (71/1920) - k5p * (17253/339200) + k6p * (22/525) - vel7 * (1/40)).
                local dVel is h * (k1a * (71/57600) - k3a * (71/16695) + k4a * (71/1920) - k5a * (17253/339200) + k6a * (22/525) - k7a * (1/40)).
                set err to max(dPos:mag / (rk45Atol + rk45Rtol * pos7:mag), dVel:mag / (rk45Atol + rk45Rtol * vel7:mag)).

                if err > 1 and h > dtMin {
                    set h to h * 0.9 * err ^ (-0.2).
                    if h < dtMin { set h to dtMin. }
                    set rejectedSteps to rejectedSteps + 1.
                } else {
                    set accept to true.
                }
            }

            set prevK7a to k7a.
            set position to pos7.
            set vel to vel7.
            set hitTime to hitTime + h.
            set steps to steps + 1.

            if err < 0.000000001 { set err to 0.000000001. }
            local factor is 0.9 * err ^ (-0.2).
            if factor < 0.2 { set factor to 0.2. }
            if factor > 5.0 { set factor to 5.0. }
            set h to h * factor.
            if h < dtMin { set h to dtMin. }
            if h > dtMax { set h to dtMax. }
        }
    } else {
        until altitude_ <= targetAltitude {
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
            set steps to steps + 1.
        }
    }

    local prevHeightAboveTarget is prevAltitude - targetAltitude.
    local curHeightAboveTarget is altitude_ - targetAltitude.
    local frac is prevHeightAboveTarget / (prevHeightAboveTarget - curHeightAboveTarget).
    set position to prevPosition + (position - prevPosition) * frac.
    set hitTime to prevHitTime + (hitTime - prevHitTime) * frac.
    print "Time to hit " + (hitTime - time:seconds) at (0, 20).

    local impactGeo is geoAtSimTime(position, hitTime).
    return lexicon(
        "impactPosition", position,
        "impactGeo", impactGeo,
        "timeToHit", hitTime - time:seconds,
        "simTimeToHit", hitTime - startUT,
        "steps", steps,
        "forceEvals", forceEvals,
        "rejectedSteps", rejectedSteps,
        "method", method
    ).
}

// Benchmark: run rk4 (current), rk45, and a fine "reference" integration from
// the same state, then report accuracy vs reference and cost for each.
function compareIntegrators {
    parameter simulationVelocity.
    parameter rk45Rtol is 0.1.
    parameter rk45Atol is 0.001.
    parameter energyStep is 80000.
    parameter dtMin is 0.1.
    parameter dtMax is 8.
    parameter refEnergyStep is 5000.
    parameter refDtMin is 0.02.
    parameter refDtMax is 1.

    local wallT0 is time:seconds.
    local rk4 is integrateTrajectory(simulationVelocity, energyStep, dtMin, dtMax, "rk4").
    local wallT1 is time:seconds.
    local rk45 is integrateTrajectory(simulationVelocity, energyStep, dtMin, dtMax, "rk45", rk45Rtol, rk45Atol).
    local wallT2 is time:seconds.
    local ref is integrateTrajectory(simulationVelocity, refEnergyStep, refDtMin, refDtMax, "rk4").
    local wallT3 is time:seconds.

    local rk4PosErr is (rk4["impactPosition"] - ref["impactPosition"]):mag.
    local rk45PosErr is (rk45["impactPosition"] - ref["impactPosition"]):mag.
    local rk4TimeErr is abs(rk4["simTimeToHit"] - ref["simTimeToHit"]).
    local rk45TimeErr is abs(rk45["simTimeToHit"] - ref["simTimeToHit"]).

    print "".
    print "--- integrator benchmark ---".
    print "reference: energyStep " + refEnergyStep + ", dt " + refDtMin + ".." + refDtMax.
    print "rk4 : pos err " + round(rk4PosErr, 1) + " m, time err " + round(rk4TimeErr, 2) + " s, steps " + rk4["steps"] + ", force evals " + rk4["forceEvals"].
    print "rk45: pos err " + round(rk45PosErr, 1) + " m, time err " + round(rk45TimeErr, 2) + " s, steps " + rk45["steps"] + ", force evals " + rk45["forceEvals"] + " (rejected " + rk45["rejectedSteps"] + ")".
    print "game-time spent: rk4 " + round(wallT1 - wallT0, 3) + " s, rk45 " + round(wallT2 - wallT1, 3) + " s, ref " + round(wallT3 - wallT2, 3) + " s".
    print "accuracy matches rk4 (rk45 err <= 1.5x rk4 err): " + (rk45PosErr <= rk4PosErr * 1.5) + " (tune rk45Rtol if not)".

    return lexicon(
        "rk4", rk4,
        "rk45", rk45,
        "reference", ref,
        "rk4PosErr", rk4PosErr,
        "rk45PosErr", rk45PosErr,
        "rk4TimeErr", rk4TimeErr,
        "rk45TimeErr", rk45TimeErr,
        "rk4WallT", wallT1 - wallT0,
        "rk45WallT", wallT2 - wallT1
    ).
}
