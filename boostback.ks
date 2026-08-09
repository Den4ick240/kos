runoncepath("0:/den4ick240kos/boosterlib.ks").

parameter landingSite.

set requiredVelocity to -ship:velocity:orbit.
set predictedTimeOfFlight to 0.
set predictedTargetVector to angleaxis(
    (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
    -ship:body:north:vector
) * (landingSite:position - ship:body:position).

set flightPathAngle to 45.

lock requiredDeltaV to requiredVelocity - ship:velocity:orbit.
lock steering to lookdirup(requiredDeltaV, ship:up:vector).
lock throttle to choose min(1.0, requiredDeltaV:mag / 20) 
if vectorangle(ship:facing:vector, requiredDeltaV) < 5 else 0.


until false {
    set predictedTimeOfFlight to getTimeOfFlight(flightPathAngle, predictedTargetVector).
    set predictedTargetVector to angleaxis(
        (360 / ship:body:rotationperiod) * predictedTimeOfFlight,
        -ship:body:north:vector
    ) * (landingSite:position - ship:body:position).

    local lowAngle is max(2, flightPathAngle - 25).
    local highAngle is min(85, flightPathAngle + 25).
    local bestDeltaV is v(0, 0, 0).

    from { local i is 0. } until i = 10 step { set i to i + 1. } do {
        local m1 is lowAngle + (highAngle - lowAngle) / 3.
        local m2 is highAngle - (highAngle - lowAngle) / 3.
        local dv1 is getRequiredDeltaVForFlightPathAngle(m1, predictedTargetVector).
        local dv2 is getRequiredDeltaVForFlightPathAngle(m2, predictedTargetVector).
        if dv1:mag <  dv2:mag {
            set highAngle to m2.
        } else {
            set lowAngle to m1.
        }
    }
    print "updated".
    set flightPathAngle to (lowAngle + highAngle) / 2.

    set requiredVelocity to getRequiredVelocityForFlightPathAngle(
        flightPathAngle,
        predictedTargetVector
    ).
 
    if requiredDeltaV:mag < 5 { 
        unlock throttle.
        unlock steering.
        print "finished boostback".
        break. 
    }

    displayPredictedHit().

    wait 0.
}
