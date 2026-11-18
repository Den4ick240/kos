
function getLocalHohmann {
    parameter mun_.
    parameter splitCount is 1.

    local home is ship:body.
    
    local ts is ship:orbit:period.
    local tm is mun_:orbit:period.
    local window is 1 / abs(1 / ts - 1 / tm).     // synodic period

    local r1 is ship:orbit:semimajoraxis.
    local r2 is mun_:orbit:semimajoraxis.
    local atx is (r1 + r2) / 2.
    local dv0 is sqrt(home:mu * (2 / r1 - 1 / atx)) - sqrt(home:mu / r1).   // Hohmann estimate

    local tTransfer is constant:PI * sqrt(atx * atx * atx / home:mu).

    local shipOmega is 2 * constant:pi / ts.
    local munOmega is 2 * constant:pi / tm.

    local relativeOmega is munOmega - shipOmega.

    local travelAngle is munOmega * tTransfer.
    local phaseRequired is constant:pi - travelAngle.

    local shipAngle is ship:longitude * constant:degtorad.
    local munAngle is mun_:longitude * constant:degtorad.

    local phaseNow is munAngle - shipAngle.

    local phaseError is phaseRequired - phaseNow.

    local waitTime is phaseError / relativeOmega.

    until waitTime < window {
        set waitTime to waitTime - window.
    }

    until waitTime >= 0 {
        set waitTime to waitTime + window.
    }

    local burnTime is time:seconds + waitTime.

    return list(burnTime, dv0, 0, 0).
}
