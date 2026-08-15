// ============================================================================
// ATTITUDE CONTROL
// ============================================================================
// Two-layer PID controller that holds the nose on a desired direction by
// writing SHIP:CONTROL pitch/yaw/roll directly (NOT kOS's steeringmanager,
// whose torque model ignores control-surface/airbrake authority).
//
//   Layer 1: attitude error (rad)   -> target angular velocity (rad/s)
//   Layer 2: rate error (rad/s)     -> control input in [-1,1]
//   The layer-2 integrator produces the constant input needed to hold against
//   a steady aero disturbance (drag torque).
//
// Public API:
//   attitudeInit()            - one-time setup (sas off, throttle 0, roll ref)
//   attitudeHold(desiredDir)  - one control step, called per frame
//
// Tuning (globals, override after runoncepath):
//   kpAng, kdAng, kpRat, kiRat, kdRat, maxOmega, pSign, ySign, rSign, rollControl
//
// Diagnostics (globals):
//   attP, attY, attR (deg), ctlP, ctlY, ctlR (-1..1),
//   tgtP, tgtY, actP, actY, actR (rad/s)

set kpAng to 1.5.   // rad/s of target rate per rad of attitude error
set kdAng to 0.3.   // anticipation term (on error change rate)
set kpRat to 0.5.   // control per rad/s of rate error
set kiRat to 0.05.  // integrator: holds constant deflection vs drag torque
set kdRat to 0.1.   // damping on rate error spikes
set maxOmega to 0.6. // layer-1 clamp: max target rate (rad/s), 0.6 = ~34 deg/s
set pSign to 1.0.    // flip to -1 if a control surface responds inverted
set ySign to 1.0.
set rSign to 1.0.
set rollControl to true.

// Layer 1: attitude error -> target angular velocity, clamped +/-maxOmega.
local angP is PIDLoop(kpAng, 0, kdAng, -maxOmega, maxOmega).
local angY is PIDLoop(kpAng, 0, kdAng, -maxOmega, maxOmega).
local angR is PIDLoop(kpAng, 0, kdAng, -maxOmega, maxOmega).
set angP:setpoint to 0.
set angY:setpoint to 0.
set angR:setpoint to 0.

// Layer 2: rate error -> control input, clamped to [-1,1].
local rateP is PIDLoop(kpRat, kiRat, kdRat, -1, 1).
local rateY is PIDLoop(kpRat, kiRat, kdRat, -1, 1).
local rateR is PIDLoop(kpRat, kiRat, kdRat, -1, 1).

// Diagnostics.
local attP to 0.
local attY to 0.
local attR to 0.
local ctlP to 0.
local ctlY to 0.
local ctlR to 0.
local tgtP to 0.
local tgtY to 0.
local tgtR to 0.
local actP to 0.
local actY to 0.
local actR to 0.

function attitudeInit {
    // Make sure nothing else fights us for the controls.
    sas off.
    lock throttle to 0.
    // Re-init PID state in case this file was already loaded for a prior launch.
    angP:reset().
    angY:reset().
    angR:reset().
    rateP:reset().
    rateY:reset().
    rateR:reset().
    print "sas " + sas + " suppressAutopilot " + config:suppressautopilot + " rollCtl " + rollControl at (0, 21).
}

function attitudeHold {
    parameter desiredDir.

    // Work in the ship-local frame: +x = starboard, +y = top, +z = fore.
    local localDesired is ship:facing:inverse * desiredDir.
    // "Control from here" on a booster often sits on a part whose fore points at
    // the TAIL, so desiredDir appears ~180 deg behind the nose. Then atan2 flips
    // through +/-180 deg and the craft thrashes trying to turn all the way around.
    // If the desired direction is behind the local fore, hold the OTHER end on it
    // instead - the craft then points the right way and the errors stay small.
    if vdot(localDesired, v(0,0,1)) < 0 {
        set localDesired to -localDesired.
    }
    local pErr is arctan2(localDesired:y, localDesired:z).   // + = nose up
    local yErr is arctan2(localDesired:x, localDesired:z).   // + = nose starboard

    // Rotation signs (kOS ANGLEAXIS convention, verified against the doc examples):
    // pitch up = -rot about starboard, yaw right = +rot about top, roll right
    // (SHIP:CONTROL:ROLL +) = +rot about fore. So pitch-up rate = -omega:x,
    // yaw-right rate = +omega:y, roll-right rate = +omega:z.
    local omegaLocal is ship:facing:inverse * ship:angularvel.

    set tgtP to -angP:update(time:seconds, pErr).
    set tgtY to -angY:update(time:seconds, yErr).

    set rateP:setpoint to tgtP.
    set rateY:setpoint to tgtY.
    set ctlP to rateP:update(time:seconds, -omegaLocal:x) * pSign.
    set ctlY to rateY:update(time:seconds, omegaLocal:y) * ySign.

    set ctlR to 0.
    set attR to 0.
    if rollControl {
        // Pure roll-rate damping: hold the current roll by commanding zero roll
        // rate. A position-hold referenced to initFacing mixes pitch/yaw into the
        // Euler :roll and fights steering; "top to world-up" (arctan2 of near-zero
        // comps) is degenerate when flying near-vertical. Rate damping alone is
        // all an axisymmetric booster needs - roll angle doesn't matter.
        set tgtR to 0.
        set rateR:setpoint to tgtR.
        set ctlR to rateR:update(time:seconds, omegaLocal:z) * rSign.
        set attR to omegaLocal:z * constant:radtodeg.
    }

    set ship:control:pitch to ctlP.
    set ship:control:yaw to ctlY.
    set ship:control:roll to ctlR.

    set attP to pErr * constant:radtodeg.
    set attY to yErr * constant:radtodeg.
    set actP to -omegaLocal:x.
    set actY to omegaLocal:y.
    set actR to omegaLocal:z.

    print "att p " + round(attP, 1) + " y " + round(attY, 1) + " r " + round(attR, 1) + " ctl " + round(ctlP, 2) + "/" + round(ctlY, 2) + "/" + round(ctlR, 2) at (0, 23).
    print "rate tgt p " + round(tgtP, 2) + " y " + round(tgtY, 2) + " act p " + round(actP, 2) + " y " + round(actY, 2) + " r " + round(actR, 2) at (0, 24).
    testTorque().
}

function testTorque {
    local torque is addons:far:AEROTORQUE().
    print "aerotorque " + torque at(0, 10).
    print "aerotorque mag " + torque:mag  at(0, 11).
}
