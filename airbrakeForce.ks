// ============================================================================
// AIRBRAKE SIMULATION LIBRARY
// ============================================================================

// 1. Load generated lexicon (make sure airbrake_db.ks is in the same folder)
IF EXISTS("0:/airbrake_db.ks") {
    RUNONCEPATH("0:/airbrake_db.ks").
} ELSE {
    print "airbrake_db not found, using fallback airbrake coeff".
    GLOBAL airbrakeCoeffOverrides IS LEXICON().
}

// 2. Physics curves (Unity FloatCurve Hermite keyframes)
GLOBAL machCdCurve IS LIST(
    LIST(0.00, 1.0000, 0.00,       0.00715953),
    LIST(0.85, 1.2500, 0.7780356,  0.7780356),
    LIST(1.10, 2.5000, 0.2492796,  0.2492796),
    LIST(5.00, 3.0000, 0,          0)
).

GLOBAL wingAoACurve IS LIST(
    LIST(0.00,       0.0100, 0.00,       0.00),
    LIST(0.3420201,  0.0600, 0.1750731,  0.1750731),
    LIST(0.50,       0.2400, 2.60928,    2.60928),
    LIST(0.7071068,  1.7000, 3.349777,   3.349777),
    LIST(1.00,       2.4000, 1.387938,   0.00)
).

// 3. Hermite Interpolation Helper
FUNCTION hermiteInterp {
    PARAMETER keys, x.
    IF x <= keys[0][0] { RETURN keys[0][1]. }
    LOCAL n IS keys:LENGTH.
    IF x >= keys[n-1][0] { RETURN keys[n-1][1]. }
    LOCAL i IS 0.
    UNTIL keys[i+1][0] >= x { SET i TO i+1. }
    LOCAL k0 IS keys[i]. LOCAL k1 IS keys[i+1].
    LOCAL dt IS k1[0] - k0[0].
    LOCAL t IS (x - k0[0]) / dt.
    LOCAL t2 IS t*t. LOCAL t3 IS t2*t.
    LOCAL h00 IS 2*t3 - 3*t2 + 1.
    LOCAL h10 IS t3 - 2*t2 + t.
    LOCAL h01 IS -2*t3 + 3*t2.
    LOCAL h11 IS t3 - t2.
    RETURN h00*k0[1] + h10*dt*k0[2] + h01*k1[1] + h11*dt*k1[3].
}

//FUNCTION getMachMultiplier { PARAMETER mach. RETURN hermiteInterp(machCdCurve, mach). }
//FUNCTION getAoAMultiplier  { PARAMETER absTerm. RETURN hermiteInterp(wingAoACurve, MIN(ABS(absTerm), 1)). }
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

FUNCTION getAirbrakeCoeff {
    PARAMETER partName.
    IF airbrakeCoeffOverrides:HASKEY(partName) { RETURN airbrakeCoeffOverrides[partName]. }
    RETURN 0.38. // Fallback
}

// ============================================================================
// ONE-TIME INITIALIZATION
// ============================================================================

GLOBAL cachedAirbrakes IS LIST().

FUNCTION initAirbrakeCache {
    cachedAirbrakes:CLEAR().
    LOCAL shipInvFacing IS SHIP:FACING:INVERSE.

    FOR p IN SHIP:PARTS {
        IF p:HASMODULE("ModuleAeroSurface") {
            LOCAL m IS p:GETMODULE("ModuleAeroSurface").
            
            // Read deployment angle once (defaulting to 45 degrees if unreadable)
            LOCAL dAngle IS 45.
            IF m:HASFIELD("deploy angle") {
                SET dAngle TO m:GETFIELD("deploy angle").
            }

            // Transform part's top vector into vessel-local space
            LOCAL localTop IS shipInvFacing * (-p:FACING:TOPVECTOR).

            // Read coefficient lookup
            LOCAL coeff IS getAirbrakeCoeff(p:NAME).

            // Store static object
            cachedAirbrakes:ADD(LEXICON(
                "coeff", coeff,
                "deployAngle", dAngle,
                "localTopVec", localTop
            )).
        }
    }
    PRINT "Airbrake Cache Initialized: " + cachedAirbrakes:LENGTH + " airbrakes stored.".
}

// Automatically build cache on script load
initAirbrakeCache().

// ============================================================================
// FAST SIMULATION FUNCTION
// ============================================================================
// simVelocity : air-relative velocity vector (m/s, world space)
// simPosition : position vector relative to body center (m, world space)
// simFacing   : ship attitude / direction at this step (Direction / Rotation)
// Returns     : predicted force vector in kN (world space)
FUNCTION predictAirbrakeForce {
    PARAMETER simVelocity, simPosition, simFacing.

    LOCAL totalForce IS V(0,0,0).
    LOCAL vMag IS simVelocity:MAG.
    
    // Quick exit if stationary or no airbrakes on craft
    IF vMag < 0.01 OR cachedAirbrakes:LENGTH = 0 { RETURN totalForce. }

    // 1. Calculate Altitude above sea level
    LOCAL bodyStruct IS SHIP:BODY.
    LOCAL altASL IS simPosition:MAG - bodyStruct:RADIUS.
    LOCAL atmStruct IS bodyStruct:ATM.

    // Exit early if outside atmosphere
    IF NOT atmStruct:EXISTS OR altASL >= atmStruct:HEIGHT OR altASL < 0 { 
        RETURN totalForce. 
    }

    // 2. Derive atmospheric properties using stock kOS ATM structure
    LOCAL pAtm IS atmStruct:ALTITUDEPRESSURE(altASL). // in atm
    LOCAL temp IS atmStruct:ALTITUDETEMPERATURE(altASL). // in Kelvin

    IF pAtm <= 0 OR temp <= 0 { RETURN totalForce. }

    LOCAL molarMass IS atmStruct:MOLARMASS.             // in kg/mol
    LOCAL pPa IS pAtm * Constant:AtmToKPa * 1000.       // convert atm -> Pa
    
    // Define universal gas constant R (8.3144626 J / (mol * K))
    LOCAL R_GAS IS 8.3144626.

    // Ideal Gas Law: rho = (P * M) / (R * T)
    LOCAL simDensity IS (pPa * molarMass) / (R_GAS * temp).

    // Speed of sound: sqrt(gamma * R_spec * T) where gamma = 1.4 for air
    LOCAL simSoundSpeed IS SQRT(1.4 * (R_GAS / molarMass) * temp).

    // 3. Aerodynamic States
    LOCAL vDir IS simVelocity:NORMALIZED.
    LOCAL mach IS vMag / simSoundSpeed.
    LOCAL q_ IS 0.5 * simDensity * vMag^2.
    LOCAL machMult IS getMachMultiplier(mach).

    // 4. Transform velocity vector into simulated vessel-local space ONCE
    LOCAL localVDir IS simFacing:INVERSE * vDir.

    // 5. Calculate force across cached airbrakes
    FOR b IN cachedAirbrakes {
        // Both vectors are now in Vessel-Local Space
        LOCAL vdotZ IS VDOT(b:localTopVec, localVDir).
        SET vdotZ TO MAX(MIN(vdotZ, 1), -1).

local ann is b:deployAngle - ARCSIN(vdotZ).
print "airbrake angle " + ann at(0, 15).
        // Deflection angle assuming fully deployed
        LOCAL deflAngle IS SIN(ann).


//LOCAL rawCd IS MIN(getAoAMultiplier(deflAngle) * machMult, 1.35).
        LOCAL cd_ IS getAoAMultiplier(deflAngle).// * machMult.
        LOCAL forceMag IS b:coeff * q_ * cd_ / 1000.

        SET totalForce TO totalForce - forceMag * vDir.
        return totalForce * 4.
    }

    RETURN totalForce.
}
