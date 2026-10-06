runoncepath("0:/den4ick240kos/prediction/boostback").
runoncepath("0:/den4ick240kos/boosterProfile/getLiveProfile").
runoncepath("0:/den4ick240kos/prediction/geo").

parameter landingSite.

local profile is getLiveProfile().

initializeBoostbackStateInitialTarget(landingSite:position - body:position).

//optimizeBoostbackState(profile, landingSite:position - body:position).
//optimizeBoostbackState(profile, landingSite:position - body:position).
//optimizeBoostbackState(profile, landingSite:position - body:position).

set requiredVelocity to optimizeBoostbackState(profile, landingSite:position - body:position):requiredVelocity.

lock requiredDeltaVLock to requiredVelocity - ship:velocity:orbit.
lock steering to lookdirup(requiredDeltaVLock, ship:up:vector).


// Calculate independent pitch and yaw deviations from the required burn vector.
// 90 minus the angle to the top/star vectors gives us the exact pitch/yaw deviation.
lock pitchErr to abs(90 - vang(ship:facing:topvector, requiredDeltaVLock)).
lock yawErr to abs(90 - vang(ship:facing:starvector, requiredDeltaVLock)).

// Calculate penalty factors from 0.0 (good) to 1.0 (bad).
// Yaw: 0 at 2 degrees, scales up to 1 at 5 degrees (range of 3).
lock yawPenalty to max(0, min(1, (yawErr - 3) / 6)).

// Pitch: 0 at 8 degrees, scales up to 1 at 12 degrees (range of 4).
lock pitchPenalty to max(0, min(1, (pitchErr - 6) / 6)).

// Take the worst of the two penalties
lock errorPenalty to max(yawPenalty, pitchPenalty).

local minThrottle is 0. // will be changed in the loop.
// Smoothly interpolate the throttle
lock baseThrottle to min(1.0, requiredDeltaVLock:mag / 20).
lock throttle to minThrottle + (baseThrottle - minThrottle) * (1 - errorPenalty).


until requiredDeltaVLock:mag < 5 {
    local boostback is optimizeBoostbackState(profile, landingSite:position - body:position).
    set requiredVelocity to boostback:requiredVelocity.

    if throttle > 0 {
        set minThrottle to 0.1.
    }
    wait 0.
}

unlock throttle.
unlock steering.
