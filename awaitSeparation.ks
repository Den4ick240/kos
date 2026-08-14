
function awaitSeparation {
    SET parentVessel TO SHIP:NAME.
    wait 2.
    //WAIT UNTIL SHIP:NAME <> parentVessel.
    PRINT "Booster separated".
}
