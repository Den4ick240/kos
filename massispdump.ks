// ============================================================================
// MASSISP_DUMP
// ============================================================================
// Reads the tag of the currently running kOS CPU and writes the craft's
// performance data to:
//   0:/den4ick240kos/massisp_<tag>.json  (machine readable, consumed by
//                                        boosterlib stage functions)
//   0:/den4ick240kos/massisp_<tag>.txt   (human readable)
//
// Contents (all measured on the craft that is CURRENTLY the active SHIP):
//   - dry / wet / current mass (metric tons)
//   - height of the craft below the current CoM (m), used for hit computations
//   - combined engine Isp and thrust sweeps over an altitude grid (via the
//     pressure profile BODY:ATM:ALTITUDEPRESSURE), using mass-flow weighting:
//       mdot_i = T_i / (Isp_i * g0)
//       shipIsp = (Sum T_i) / ((Sum mdot_i) * g0)
//   - derived total delta-V (wet->dry) and current delta-V using the vacuum Isp
//   - a drag table: combined aero force (kN) over an altitude x airspeed grid,
//     sampled from the FAR add-on (ADDONS:FAR:AEROFORCEAT) with the velocity
//     vector along -fore, i.e. the craft moving tail-first (the landing /
//     re-entry attitude). This is the same convention integrateStageTrajectory
//     uses to apply drag.
//
// IMPORTANT: run this with the craft ALONE on the pad, stacked in its normal
// landing orientation (nose up). The dump then captures only that vehicle's
// engines, mass and drag, which is what the pre-separation RTLS watchdog on
// the booster CPU needs to simulate the stage after it separates.

clearscreen.

// --- 1. Current processor's part tag --------------------------------------
local procTag is core:tag.
print "CPU part tag: " + procTag.

// --- 2. Build a file stem from the tag (lowercase, alnum + underscore). ----
local fname is "".
for ch in procTag {
    if (ch >= "a" and ch <= "z") or (ch >= "A" and ch <= "Z") or (ch >= "0" and ch <= "9") or ch = "_" {
        set fname to fname + ch.
    } else {
        set fname to fname + "_".
    }
}
set fname to fname:toLower.
if fname:trim = "" { set fname to "cpu". }

// Start from clean files so re-runs don't append.
local jsonPath is "0:/den4ick240kos/massisp_" + fname + ".json".
local txtPath  is "0:/den4ick240kos/massisp_" + fname + ".txt".
if exists(jsonPath) { deletepath(jsonPath). }
if exists(txtPath)  { deletepath(txtPath). }

// --- 3. Masses -------------------------------------------------------------
local dryMass is ship:drymass.
local wetMass is ship:wetmass.
local curMass is ship:mass.

log "CPU tag: " + procTag to txtPath.
log "Ship: " + ship:name to txtPath.
log "Dry mass (t): " + round(dryMass, 3) to txtPath.
log "Wet mass (t): " + round(wetMass, 3) to txtPath.
log "Current mass (t): " + round(curMass, 3) to txtPath.

// --- 4. Height of the craft below the current CoM --------------------------
// Physical distance from the vessel coordinate origin (CoM) down to the
// bottom-most point of the craft along the ship's fore/aft axis. Stable in any
// orientation, same helper as boosterlib:getHeightFromOriginToBottom.
local vesselHeight is 0.
local fore is ship:facing:forevector.
local aftRay is -fore.
local farthestPt is 0.
list parts in _dpParts.
for p in _dpParts {
    local b is p:bounds.
    local corner is b:furthestcorner(aftRay).
    local d is vdot(corner, fore).
    if d < farthestPt { set farthestPt to d. }
}
set vesselHeight to -farthestPt.

log "Height below CoM (m): " + round(vesselHeight, 2) to txtPath.
log "" to txtPath.

// --- 5. Active engine set (fall back to all engines if none are ignited). --
local engs is ship:engines.
local activeEngs is list().
local anyIgnited is false.
for eng in engs {
    if eng:ignition {
        activeEngs:add(eng).
        set anyIgnited to true.
    }
}
if not anyIgnited { set activeEngs to engs. }

local g0 is constant:g0.

// Thrust of one engine at a pressure: use POSSIBLETHRUSTAT, fall back to
// MAXTHRUSTAT (engine disabled / old kOS builds report zero on the pad).
function engineThrustAt {
    parameter eng.
    parameter pressure.
    local t is 0.
    try { set t to eng:possiblethrustat(pressure). } catch { }
    if t <= 0 {
        try { set t to eng:maxthrustat(pressure). } catch { }
    }
    return t.
}

function combinedThrustAt {
    parameter pressure.
    local total is 0.
    for eng in activeEngs {
        set total to total + engineThrustAt(eng, pressure).
    }
    return total.
}

function combinedIspAt {
    parameter pressure.
    local totalThrust is 0.
    local totalMassFlow is 0.
    for eng in activeEngs {
        local thrust is engineThrustAt(eng, pressure).
        local isp is 0.
        try { set isp to eng:ispat(pressure). } catch { }
        if isp <= 0 {
            try { set isp to eng:isp. } catch { }
        }
        if thrust > 0 and isp > 0 {
            set totalThrust to totalThrust + thrust.
            set totalMassFlow to totalMassFlow + thrust / (isp * g0).
        }
    }
    if totalMassFlow > 0 { return totalThrust / (totalMassFlow * g0). }
    return 0.
}

// --- 6. Sweep altitudes up to the top of the atmosphere --------------------
local body_ is ship:body.
local atmosTop is 0.
if body_:atm:exists { set atmosTop to body_:atm:height. }

local altBins is list().
if body_:atm:exists {
    local a is 0.
    until a > atmosTop {
        altBins:add(round(a)).
        if a < 1000 { set a to a + 250. }
        else if a < 10000 { set a to a + 1000. }
        else if a < 25000 { set a to a + 2500. }
        else { set a to a + 5000. }
    }
    altBins:add(round(atmosTop) + 1000). // beyond atmosphere: p = 0 (vacuum)
} else {
    altBins:add(0).
    altBins:add(10000).
    altBins:add(300000).
}

local ispTable is list().
local thrustTable is list().
log "altitude(m),pressure(atm),ship_isp(s),thrust(kN)" to txtPath.
for alt in altBins {
    local pressure is 0.
    if body_:atm:exists and alt <= atmosTop {
        set pressure to body_:atm:altitudepressure(alt).
    }
    local isp is combinedIspAt(pressure).
    local thr is combinedThrustAt(pressure).
    ispTable:add(round(isp, 1)).
    thrustTable:add(round(thr, 1)).
    log "alt=" + alt + ",p=" + round(pressure, 6) + ",isp=" + round(isp, 1) + ",thrust=" + round(thr, 1) to txtPath.
    print "alt " + alt + " m  p " + round(pressure, 4) + " atm  Isp " + round(isp, 1) + " s  T " + round(thr, 1) + " kN".
}

// --- 7. Derived delta-V (vacuum Isp ideal rocket equation) ------------------
local ispVac is combinedIspAt(0).
local dvTotal is 0.
local dvNow is 0.
if ispVac > 0 and dryMass > 0 {
    if wetMass > dryMass { set dvTotal to ispVac * g0 * ln(wetMass / dryMass). }
    if curMass > dryMass  { set dvNow to ispVac * g0 * ln(curMass / dryMass). }
}

log "" to txtPath.
log "Isp vacuum (s): " + round(ispVac, 1) to txtPath.
log "Total dV wet->dry (m/s): " + round(dvTotal, 1) to txtPath.
log "Current dV cur->dry (m/s): " + round(dvNow, 1) to txtPath.

// --- 8. Drag table: altitude x airspeed grid, force in kN ------------------
local speedBins is list().
local sb is 0.
until sb > 2600 {
    speedBins:add(round(sb)).
    if sb < 400 { set sb to sb + 50. }
    else if sb < 1200 { set sb to sb + 100. }
    else { set sb to sb + 200. }
}

// Convention: velocity along -fore (craft moving tail-first). On the pad the
// stage sits nose up, so -fore points down = same attitude as the descent burn.
local dragDir is -ship:facing:forevector.

local dragOk is false.
try { if addons:far:apiinstalled { set dragOk to true. } } catch { }

log "" to txtPath.
log "# DRAG_TABLE (kN), altitude(m) rows x airspeed(m/s) columns" to txtPath.
log "speed_bins(m/s):" to txtPath.
local sIdx is 0.
local sbLine is "".
for sbv in speedBins {
    set sbLine to sbLine + round(sbv, 0).
    if sIdx < speedBins:length - 1 { set sbLine to sbLine + ",". }
    set sIdx to sIdx + 1.
}
log sbLine to txtPath.

local dragRows is list().
local anyDrag is false.
for alt in altBins {
    local row is list().
    for s in speedBins {
        local kf is 0.
        if dragOk {
            try { set kf to addons:far:aeroforceat(alt, dragDir * s):mag. } catch { set kf to 0. }
        }
        set kf to kf / 1000.0. // N -> kN
        if kf > 0 { set anyDrag to true. }
        row:add(round(kf, 4)).
    }
    dragRows:add(row).
}

local aIdx is 0.
for row in dragRows {
    local line is "row alt=" + round(altBins[aIdx], 0) + ":".
    local cIdx is 0.
    for v in row {
        set line to line + round(v, 4).
        if cIdx < row:length - 1 { set line to line + ",". }
        set cIdx to cIdx + 1.
    }
    log line to txtPath.
    set aIdx to aIdx + 1.
}

if not anyDrag {
    print "WARNING: drag table is all-zero (FAR unavailable, or no aero force on the pad)".
}

// --- 9. Serialize everything the stage sim needs to one JSON file ----------
local stageData is lex(
    "tag", fname,
    "shipName", ship:name,
    "engineCount", activeEngs:length,
    "dryMass", round(dryMass, 3),
    "wetMass", round(wetMass, 3),
    "curMass", round(curMass, 3),
    "height", round(vesselHeight, 2),
    "ispVac", round(ispVac, 1),
    "dvTotal", round(dvTotal, 1),
    "dvNow", round(dvNow, 1),
    "atmosTop", round(atmosTop, 1),
    "g0", g0,
    "altBins", altBins,
    "speedBins", speedBins,
    "ispTable", ispTable,
    "thrustTable", thrustTable,
    "dragTable", dragRows,
    "luckySuffix", "boosters"
).

writejson(stageData, jsonPath).
print "Wrote " + jsonPath.
print "Wrote " + txtPath.