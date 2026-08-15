local assumedControlSurfaceRotationSpeed is 2.0 // assume control surface can go from -1 to 1 in one second
local minExcitation is 0.015. // ignore secant slope if control barely moved last tick
local gradFilterAlpha is 0.3.   // 0 = trust old slope estimate, 1 = trust newest sample fully

local lastTime is time:seconds.
local commandGradient is v(4000, 4000, 1500).
local prevTorque is getTorque().
local prevCommand is v(ship:control:pitch, ship:control:yaw, ship:control:roll).

function clamp {
    parameter value, low, high.
    if value < low { return low. }
    if value > high { return high. }
    return value.
}

function clampVector {
    parameter value, low, high.
    return v(
        clamp(value:x, low, high),
        clamp(value:y, low, high),
        clamp(value:z, low, high)
    ).
}

function divideVector {
    parameter va, vb.
    return v(va:x / vb:x, va:y / vb:y, va:z / vb:z).
}

function getTorque {
    local worldTorque is addons:far:aerotorque().
    return v(
        vdot(worldTorque, ship:facing:starvector),
        vdot(worldTorque, ship:facing:topvector),
        vdot(worldTorque, ship:facing:forevector)
    ).
}

function getMaxRatePerTick {
    local currentTime is time:seconds.
    until currentTime <> lastTime {
        wait 0.
        set currentTime to time:seconds
    }
    local dt is currentTime - lastTime.
    set lastTime to currentTime.
    return assumedControlSurfaceRotationSpeed * dt.
}

function updateGradient {
    parameter currentTorque, currentCommand.
    local dCommand is currentCommand - prevCommand.
    local dTorque is currentTorque - prevTorque.
    local newGradient is commandGradient * (1 - gradFilterAlpha) + 
        divideVector(dTorque, dCommand) * gradFilterAlpha.
    if abs(dCommand:x) > minExcitation {
        set commandGradient:x to newGradient:x.
    }
    if abs(dCommand:y) > minExcitation {
        set commandGradient:y to newGradient:y.
    }
    if abs(dCommand:z) > minExcitation {
        set commandGradient:z to newGradient:z.
    }
}

function updateTorque {
    parameter desiredPitch, desiredYaw, desiredRoll.
    local maxRate is getMaxRatePerTick().

    local currentTorque is getTorque().
    local currentCommand is v(ship:control:pitch, ship:control:yaw, ship:control:roll).

    updateGradient(currentTorque, currentCommand).

    local desiredTorque is v(desiredPitch, desiredYaw, desiredRoll).

    local commandIncrement is clampVector(
        divideVector(desiredTorque - currentTorque, commandGradient),
        -maxRate,
        maxRate
    ).

    set prevCommand to clampVector(
        currentCommand + commandIncrement,
        -1.0,
        1.0
    ).

    set ship:control:pitch to prevCommand:x.
    set ship:control:yaw to prevCommand:y.
    set ship:control:roll to prevCommand:z.

    set prevTorque to currentTorque.
}
