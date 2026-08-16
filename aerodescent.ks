runoncepath("0:/den4ick240kos/boosterlib.ks").
runoncepath("0:/den4ick240kos/guidance.ks").
runoncepath("0:/den4ick240kos/holdTorque.ks").

parameter landingSite.

local maxRate is 0.6.
local rateKp is 0.025.
local rateKd is 0.01.
local torqueKp is 10.
local torqueKd is 1.

local pitchAngleToRate is PIDLoop(rateKp, 0, rateKd, -maxRate, maxRate).
local yawAngleToRate is PIDLoop(rateKp, 0, rateKd, -maxRate, maxRate).
local rollAngleToRate is PIDLoop(0.05, 0, 0.005, -maxRate, maxRate).
set pitchAngleToRate:setpoint to 0.
set yawAngleToRate:setpoint to 0.
set rollAngleToRate:setpoint to 0.

local maxTorque is 1.
local pitchRateToTorque is PIDLoop(torqueKp, 0, torqueKd, -maxTorque, maxTorque).
local yawRateToTorque is PIDLoop(torqueKp, 0, torqueKd, -maxTorque, maxTorque).
local rollRateToTorque is PIDLoop(torqueKp, 0, torqueKd, -maxTorque, maxTorque).

set rollRateToTorque:setpoint to 0.

local desiredDir is srfretrograde:forevector.

local desiredRate is v(1, 0, 0).

guidanceInit(landingSite).

when true then {
    local sec is time:seconds.
    local localDesired is ship:facing:inverse * desiredDir.
    local pitchErr is arctan2(localDesired:y, localDesired:z).   // + = nose up
    local yawErr is arctan2(localDesired:x, localDesired:z).   // + = nose starboard
    local rollErr TO 90 - VANG(SHIP:FACING:STARVECTOR, UP:VECTOR).

    print "pitch error" + pitchErr at (0, 0).
    
    local angularVelLocal is ship:facing:inverse * ship:angularvel.

    set pitchRateToTorque:setpoint to pitchAngleToRate:update(sec, pitchErr).
    set yawRateToTorque:setpoint to -yawAngleToRate:update(sec, yawErr).
    set rollRateToTorque:setpoint to rollAngleToRate:update(sec, rollErr).

    set desiredPitch to -pitchRateToTorque:update(sec, angularVelLocal:x).    
    set desiredYaw to yawRateToTorque:update(sec, angularVelLocal:y).
    set desiredRoll to -rollRateToTorque:update(sec, angularVelLocal:z).

    if true {
        set ship:control:pitch to desiredPitch.
        set ship:control:yaw to desiredYaw.
        set ship:control:roll to desiredRoll.
    } else {
        updateTorque(desiredPitch, desiredYaw, desiredRoll).
    }
    return true.
}

until false {
    set desiredDir to guidanceUpdate().
    //boot/
//set desiredDir to srfretrograde:forevector.
    wait 0.
}

until false {
wait 1.
}
