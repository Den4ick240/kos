
function geoAtSimTimeProvider {
    local bodyPos is body:position.
    local northDir is (latlng(90, 0):position - bodyPos):normalized.
    local lon0Dir is (latlng(0, 0):position - bodyPos):normalized.
    local lon90Dir is (latlng(0, 90):position - bodyPos):normalized.
    local degPerSec is 360 / body:rotationperiod.
    local calculationStartTime is time:seconds.

    return {
        parameter pos, targetTime.
        local posMag is pos:mag.
        local lat_ is arcsin(vdot(pos, northDir) / posMag).
        local lon_ is arctan2(vdot(pos, lon90Dir), vdot(pos, lon0Dir)) - degPerSec * (targetTime - calculationStartTime).
        until lon_ <= 180 { set lon_ to lon_ - 360. }
        until lon_ >= -180 { set lon_ to lon_ + 360. }

        return latlng(lat_, lon_).
    }.
}


function getBodySnapshot {
    wait 0. // make sure all things below get written down in a single frame
    local bodyPos is body:position.
    local northDir is (latlng(90, 0):position - bodyPos):normalized.
    local lon0Dir is (latlng(0, 0):position - bodyPos):normalized.
    local lon90Dir is (latlng(0, 90):position - bodyPos):normalized.
    local degPerSec is 360 / body:rotationperiod.
    local bodyR is body:radius.
    local snapshotTime is time:seconds.

    return lex(
        "latLonToVector", {
            parameter latlon, atTime.

            local lat is latLon:lat.
            local lng is latlon:lng + degPerSec * (atTime - snapshotTime).

            return (
                lon0Dir * cos(lat) * cos(lng) +
                lon90Dir * cos(lat) * sin(lng) +
                northDir * sin(lat)
            ) * (bodyR + latlon:terrainHeight).
        },
        "rotateVectorInTime", {
            parameter targetVector, atTime.
            return angleaxis(degPerSec * (atTime - snapshotTime), northDir) * targetVector.
        },
        "vectorAtTimeToLatLon", {
            parameter targetVector, atTime.

            local targetMag is targetVector:mag.
            local lat is arcsin(vdot(targetVector, northDir) / posMag).
            local lng is arctan2(vdot(targetVector, lon90Dir), vdot(targetVector)).
            return latlng(lat, lng).
        }
    ).
}



