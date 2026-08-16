runoncepath("0:/den4ick240kos/boosterlib.ks").

        print "coasting".
    until ship:altitude < body:atm:height * 0.6 {
        displayPredictedHit().
        lock steering to srfretrograde.
        lock throttle to 0.
        wait 1.
        break.
    }
        print "finished coasting".

    unlock steering.
    unlock throttle.
