runoncepath("0:/den4ick240kos/boosterlib.ks").
runoncepath("0:/den4ick240kos/guidance.ks").
runoncepath("0:/den4ick240kos/holdTorque.ks").
runoncepath("0:/den4ick240kos/surfaces.ks").

COPYPATH("0:/den4ick240kos/landingburn.ks", "1:/den4ick240kos/landingburn.ks").
COPYPATH("0:/den4ick240kos/apdg.ks", "1:/den4ick240kos/apdg.ks").

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
local hitTime is 0.

local desiredRate is v(1, 0, 0).
local isAerodescent is true.

guidanceInit(landingSite).
surfaceFlipInit().

set steeringmanager:yawtorquefactor to 0.4.
set steeringmanager:pitchtorquefactor to 0.4.
when isAerodescent then {
    local sec is time:seconds.
    lock steering to lookdirup(desiredDir, ship:facing:topvector).
    return true.
}

when (ship:altitude < guidBurnAltitude) then {
    //print "REAL burn start: alt " + round(ship:altitude) + " vsurf " + round(ship:velocity:surface:mag, 1) + " guidBurnAlt " + round(guidBurnAltitude) at (0, 26).
    set isAerodescent to false.
    SET SHIP:CONTROL:NEUTRALIZE to True.
}

until not isAerodescent {
    local aa is guidanceUpdate().
    set desiredDir to aa["a"].
    set hitTime to aa["b"].
    if ship:altitude < guidBurnAltitude {
        break.
    }
    wait 0.
}

RUNPATH("1:/den4ick240kos/apdg.ks", landingSite, hitTime).
