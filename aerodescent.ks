// ============================================================================
// ATMOSPHERIC DESCENT - MAIN LOOP
// ============================================================================
// Orchestrates the descent:
//   - guidance.ks  : predicts the impact point and steers retrograde onto the
//                    landing site, producing desiredDir.
//   - attitude.ks  : holds the nose on desiredDir by writing raw controls.
// This file only wires the two together each frame and prints diagnostics.

runoncepath("0:/den4ick240kos/boosterlib.ks").
runoncepath("0:/den4ick240kos/guidance.ks").
runoncepath("0:/den4ick240kos/attitude.ks").

parameter landingSite.

// ---- Tuning overrides (optional) ----
// The libraries above set the defaults; tweak anything here, e.g.:
// set maxOmega to 0.5.
// set pSign to -1.
// set gMaxSteer to 30.

guidanceInit(landingSite).
attitudeInit().

until false {
    local desiredDir is guidanceUpdate().

    attitudeHold(desiredDir).


    wait 0.
}
