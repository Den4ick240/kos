runoncepath("0:/den4ick240kos/selectlandingsite.ks").
runoncepath("0:/den4ick240kos/awaitSeparation.ks").

PRINT "Booster script running".

awaitSeparation().
PRINT "Booster separated".

SET CONFIG:IPU TO 2000. // Default is 150

set landingSite to selectLandingSite().

SET SHIP:CONTROL:NEUTRALIZE to True.
//RUNPATH("0:/den4ick240kos/boostback/boostback_phase1.ks", landingSite).
//RUNPATH("0:/den4ick240kos/boostback/boostback.ks", landingSite).
//RUNPATH("0:/den4ick240kos/coast.ks").
RUNPATH("0:/den4ick240kos/aerodescent.ks", landingSite).
