runoncepath("0:/den4ick240kos/selectlandingsite.ks").
runoncepath("0:/den4ick240kos/boosterProfile/loadProfile.ks").
runoncepath("0:/den4ick240kos/awaitSeparation.ks").
runoncepath("0:/den4ick240kos/saveLandingSite").
runoncepath("0:/den4ick240kos/boostback/bb").
runoncepath("0:/den4ick240kos/aerodescent.ks").

SET SHIP:CONTROL:NEUTRALIZE to True.
SET CONFIG:IPU TO 2000.

local landingSite is 0.

if ship:modulesnamed("kOSProcessor"):length > 1 {    

    saveLandingSite().
    set landingSite to readLandingSite().


    runpath("0:/den4ick240kos/booster/separateForLanding.ks", landingSite, loadboosterprofile()).
    clearscreen.
    print "Awaiting separation".
    awaitSeparation().
    lock throttle to 0.
}  else {
 set landingSite to readLandingSite().
}

rcs on.
brakes on.
boostbackGuidance (landingSite).
aerodescentGuidance(landingSite).
