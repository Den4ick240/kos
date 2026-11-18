
function getSelectedWaypoint {
    for wp in allwaypoints() {
        if wp:isSelected {
            return wp.
        }
    }

    print "no selected waypoint".
}
