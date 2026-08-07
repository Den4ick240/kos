function coast {
    lock steering to ship:velocity:surface:retrograde.

    until ship:altitude < body:atm:height * 0.6 {
        wait 1.
    }

    unlock steering.
}
