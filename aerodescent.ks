runoncepath("0:/den4ick240kos/boosterlib.ks").

local lastVelocity is ship:velocity:orbit.
local lastTime is time:seconds.

lock steering to srfretrograde.


until false {
    displayPredictedHit().
    local currTime is time:seconds.
    local currVelocity is ship:velocity:orbit.
    local dt is currTime - lastTime.
    if dt <> 0 { 
    print "actual acceleration " + ((currVelocity - lastVelocity) / dt):mag at (0, 7).

    print "facing " + ship:facing:forevector at (0, 16).

    set anArrow3 to vecdraw(
    {return ship:position.},
    (currVelocity - lastVelocity) / dt,
    rgb(0,0,1),
    "actual",
    1.0,
    true,0.2,true,true
    ).
    set lastVelocity to currVelocity.
    set lastTime to currTime.
    }
    wait 0.
}
