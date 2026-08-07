runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.

set predictedTimeOfFlight to 0.

set flightPathAngle to 45.


until false {
    set predictedTimeOfFlight to getTimeOfFlight(flightPathAngle, predictedTargetVector).
    set predictedTargetVector to angleaxis(
        (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
        -ship:body:north:vector
    ) * (landingSite:position - ship:body:position).

    local lowAngle is max(2, flightPathAngle - 25).
    local highAngle is min(85, flightPathAngle + 25).

    from { local i is 0. } until i = 10 step { set i to i + 1. } do {
        local m1 is lowAngle + (highAngle - lowAngle) / 3.
        local m2 is highAngle - (highAngle - lowAngle) / 3.
        if getRequiredDeltaVForFlightPathAngle(m1, predictedTargetVector):mag <  getRequiredDeltaVForFlightPathAngle(m2, predictedTargetVector):mag {
            set highAngle to m2.
        } else {
            set lowAngle to m1.
        }
    }
    print "updated".
    set flightPathAngle to (lowAngle + highAngle) / 2.

    set requiredDeltaV to getRequiredDeltaVForFlightPathAngle(
        flightPathAngle,
        predictedTargetVector
    ).

    lock steering to lookdirup(requiredDeltaV, ship:up:vector).

    if vectorangle(ship:facing:vector, requiredDeltaV) < 5 {
        lock throttle to min(1.0, requiredDeltaV:mag / 20).
    } else {
        lock throttle to 0.
    }

    if requiredDeltaV:mag < 1 { 
        lock throttle to 0.
        print "finished boostback".
        break. 
    }

    wait 0.
}
