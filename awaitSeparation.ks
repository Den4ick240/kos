
function awaitSeparation {
    wait until ship:modulesnamed("kOSProcessor"):length = 1.
    PRINT "Booster separated".
}
