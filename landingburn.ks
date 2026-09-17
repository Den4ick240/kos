parameter lbSite is 0.
if lbSite = 0 {
    set lbSite to guidSite.
}

set lbDistKP to 0.85.
set lbDistKI to 0.0.
set lbDistKD to 0.4.
set lbVelKP to 1.0.
set lbVelKI to 0.00.
set lbVelKD to 1.5.
set lbMaxHSpeed to 10.
set lbMaxTilt to 15.
set lbKillWindow to 0.1.

local distDownrangePID is PIDLoop(lbDistKP, lbDistKI, lbDistKD, -lbMaxHSpeed, lbMaxHSpeed).
local distCrossrangePID is PIDLoop(lbDistKP, lbDistKI, lbDistKD, -lbMaxHSpeed, lbMaxHSpeed).
local velDownrangePID is PIDLoop(lbVelKP, lbVelKI, lbVelKD, -lbMaxTilt, lbMaxTilt).
local velCrossrangePID is PIDLoop(lbVelKP, lbVelKI, lbVelKD, -lbMaxTilt, lbMaxTilt).
set distDownrangePID:setpoint to 0.
set distCrossrangePID:setpoint to 0.
set velDownrangePID:setpoint to 0.
set velCrossrangePID:setpoint to 0.

function getThrottle {
    local bodyPos is ship:body:position.
    local g is ship:body:mu / (ship:position - ship:body:position):sqrmagnitude.
    local facingDotUp is vdot(ship:facing:forevector, ship:up:vector).

    local desiredAcc is 0.
    local altRemaining is max(ship:bounds:bottomaltradar, 1).
    local desiredAcc is (-abs(verticalspeed) * verticalspeed * 0.5 / altRemaining + g) / facingDotUp.
    return max(
        0,
        min(
            1,
            ship:mass * desiredAcc / ship:availablethrust()
        )
    ).
}

function getGoalPosition {
    return lbSite:altitudeposition(lbSite:terrainheight).
}

function getTiltSteering {
    local upV is ship:up:vector.
    local errVec is vectorExclude(upV, getGoalPosition() - ship:position).

    local dnDir is errVec:normalized.
    if errVec:mag < 0.001 {
        set dnDir to vectorExclude(upV, ship:facing:forevector):normalized.
    }
    if dnDir:mag < 0.001 {
        set dnDir to ship:facing:starvector.
    }
    local crDir is vcrs(upV, dnDir):normalized.

    local posDn is vdot(errVec, dnDir).
    local posCr is vdot(errVec, crDir).

    local timeToGo is 99.
    if ship:verticalspeed < -0.01 {
        set timeToGo to max(ship:bounds:bottomaltradar, 0) / (-ship:verticalspeed).
        if timeToGo > 99 { set timeToGo to 99. }
    }
    local killFactor is timeToGo / lbKillWindow.
    if killFactor > 1 { set killFactor to 1. }
    if killFactor < 0 { set killFactor to 0. }

    local wantDn is -distDownrangePID:update(time:seconds, posDn) * killFactor.
    local wantCr is -distCrossrangePID:update(time:seconds, posCr) * killFactor.

    local hVel is vectorExclude(upV, ship:velocity:surface).
    local actDn is vdot(hVel, dnDir).
    local actCr is vdot(hVel, crDir).

    local tiltDn is velDownrangePID:update(time:seconds, wantDn - actDn).
    local tiltCr is velCrossrangePID:update(time:seconds, wantCr - actCr).

    local tiltVec is dnDir * -tiltDn + crDir * -tiltCr.
    local tiltMag is tiltVec:mag.
    if tiltMag > lbMaxTilt { set tiltMag to lbMaxTilt. }

    local fore is upV.
    if tiltMag > 0.001 {
        local tiltDir is tiltVec:normalized.
        local axis is vcrs(upV, tiltDir):normalized.
        if axis:mag > 0.001 {
            set fore to angleaxis(tiltMag, axis) * upV.
        }
    }

    print "LB d " + round(posDn, 0) + "/" + round(posCr, 0) + " want " + round(wantDn, 1) + "/" + round(wantCr, 1) + " act " + round(actDn, 1) + "/" + round(actCr, 1) + " tilt " + round(tiltDn, 1) + "/" + round(tiltCr, 1) + " ttg " + round(timeToGo, 1) at (0, 22).
    return lookdirup(fore, ship:facing:topvector).
}

lock throttle to getThrottle().
lock steering to getTiltSteering().

until ship:status = "LANDED" {
    wait 0.
}

lock throttle to 0.
wait 3.
