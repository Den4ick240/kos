
function awaitSeparation {
    SET parentVessel TO SHIP:NAME.
    wait 5.
    //WAIT UNTIL SHIP:NAME <> parentVessel.
    PRINT "Booster separated".
}
