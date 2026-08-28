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

lock throttle to getThrottle().
lock steering to srfretrograde.
until ship:verticalspeed > -10 {
    wait 0.
}
lock steering to lookdirup(up:vector, facing:topvector).

until ship:status = "LANDED" {
    wait 0.
}

lock throttle to 0.
wait 3.
