// ============================================================================
// MASSISP_DUMP
// ============================================================================
// Reads the tag of the currently running kOS CPU and writes a data file to
//   0:/den4ick240kos/massisp_<tag>.txt
// containing:
//   - vessel dry mass   (SHIP:DRYMASS, metric tons)
//   - vessel wet mass   (SHIP:WETMASS, metric tons)
//   - current mass      (SHIP:MASS, metric tons)
//   - combined ship Isp of the active engine set across a sweep of altitudes.
//
// The pressure at each altitude comes from BODY:ATM:ALTITUDEPRESSURE(alt).
// The per-engine Isp comes from Engine:ISPAT(pressure), combined by mass flow:
//   mdot_i = T_i / (Isp_i * g0)
//   shipIsp = (Sum T_i) / ((Sum mdot_i) * g0)
// Below the atmosphere the last row uses p = 0, i.e. the vacuum Isp.

clearscreen.

// --- 1. Current processor's part tag --------------------------------------
local procTag is core:tag.
print "CPU part tag: " + procTag.

// --- 2. Build a file name from the tag (lowercase, alnum + underscore). ----
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

local fpath is "0:/den4ick240kos/massisp_" + fname + ".txt".

// Start from a clean file so re-runs don't append.
if exists(fpath) { deletepath(fpath). }

// --- 3. Masses -------------------------------------------------------------
local dryMass is ship:drymass.
local wetMass is ship:wetmass.
local curMass is ship:mass.

log "CPU tag: " + procTag to fpath.
log "Ship: " + ship:name to fpath.
log "Dry mass (t): " + round(dryMass, 3) to fpath.
log "Wet mass (t): " + round(wetMass, 3) to fpath.
log "Current mass (t): " + round(curMass, 3) to fpath.
log "" to fpath.
log "altitude(m),pressure(atm),ship_isp(s)" to fpath.

// --- 4. Active engine set (fall back to all engines if none are ignited). --
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

function combinedIspAt {
    parameter pressure.
    local totalThrust is 0.
    local totalMassFlow is 0.
    for eng in activeEngs {
        local thrust is eng:possiblethrustat(pressure).
        if thrust > 0 {
            local isp is eng:ispat(pressure).
            if isp > 0 {
                set totalThrust to totalThrust + thrust.
                set totalMassFlow to totalMassFlow + thrust / (isp * g0).
            }
        }
    }
    if totalMassFlow > 0 { return totalThrust / (totalMassFlow * g0). }
    return 0.
}

// --- 5. Sweep altitudes up to the top of the atmosphere --------------------
local body_ is ship:body.
local atmosTop is 0.
if body_:atm:exists { set atmosTop to body_:atm:height. }

local altList is list().
if body_:atm:exists {
    local a is 0.
    until a > atmosTop {
        altList:add(round(a)).
        if a < 1000 { set a to a + 250. }
        else if a < 10000 { set a to a + 1000. }
        else if a < 25000 { set a to a + 2500. }
        else { set a to a + 5000. }
    }
    altList:add(round(atmosTop) + 1000). // beyond atmosphere: p = 0 (vacuum)
} else {
    altList:add(0).
    altList:add(10000).
    altList:add(300000).
}

for alt in altList {
    local pressure is 0.
    if body_:atm:exists and alt <= atmosTop {
        set pressure to body_:atm:altitudepressure(alt).
    }
    local isp is combinedIspAt(pressure).
    log "alt=" + alt + ",p=" + round(pressure, 6) + ",isp=" + round(isp, 1) to fpath.
    print "alt " + alt + " m  p " + round(pressure, 4) + " atm  Isp " + round(isp, 1) + " s".
}

print "Wrote " + fpath.