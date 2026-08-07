runoncepath("0:/den4ick240kos/selectlandingsite.ks").
runoncepath("0:/den4ick240kos/awaitSeparation.ks").

PRINT "Booster script running".

awaitSeparation().
PRINT "Booster separated".

SET CONFIG:IPU TO 4000. // Default is 150

set landingSite to selectLandingSite().

RUNPATH("0:/den4ick240kos/boostback.ks", landingSite).
RUNPATH("0:/den4ick240kos/coast.ks").
RUNPATH("0:/den4ick240kos/aerodescent.ks").
