runoncepath("0:/den4ick240kos/selectlandingsite.ks").
runoncepath("0:/den4ick240kos/awaitSeparation.ks").

SET SHIP:CONTROL:NEUTRALIZE to True.
SET CONFIG:IPU TO 2000.

clearscreen.
print "Awaiting separation".
awaitSeparation().

clearscreen.
print "Selecting landing site".
set landingSite to selectLandingSite().

//RUNPATH("0:/den4ick240kos/boostback/boostback_phase1.ks", landingSite).
//RUNPATH("0:/den4ick240kos/boostback/boostback.ks", landingSite).
//RUNPATH("0:/den4ick240kos/coast.ks").
RUNPATH("0:/den4ick240kos/aerodescent.ks", landingSite).
