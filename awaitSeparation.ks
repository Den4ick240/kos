
function awaitSeparation {
    SET parentVessel TO SHIP:NAME.
    wait 0.
    //WAIT UNTIL SHIP:NAME <> parentVessel.
    PRINT "Booster separated".
}
