runoncepath(scriptpath():parent + "/integrateTable").

function loadboosterprofile {
    parameter profilePath.

    local profile is readjson(profilePath).

    local altitudeSteps is profile["altitudeSteps"].
    local speedSteps is profile["speedSteps"].
    local dryMass is profile["dryMass"].
    local wetMass is profile["wetMass"].
    local heightFromOriginToBottom is profile["heightFromOriginToBottom"].
    local ispTable is profile["ispTable"].
    local thrustTable is profile["thrustTable"].
    local massFlow is profile["massFlow"].
    local aeroforceTable is profile["retrogradeAeroforceTable"].

    local altitudeSegments is buildSegments(altitudeSteps).
    local speedSegments is buildSegments(speedSteps).

    return lex(
        "dryMass", dryMass,
        "wetMass", wetMass,
        "heightFromOriginToBottom", heightFromOriginToBottom,

        "ispat", {
            parameter a.
            return interpolate1dTable(ispTable, a, altitudeSegments).
        },

        "thrustat", {
            parameter a.
            return interpolate1dTable(thrustTable, a, altitudeSegments).
        },

        "massflow", {
            return massFlow.
        },

        "retrogradeAeroforceat", {
            parameter a, speed.
            return interpolate2dTable(aeroforceTable, a, speed, altitudeSegments, speedSegments).
        }
    ).
}
