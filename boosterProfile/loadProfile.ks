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
    local massFlowTable is profile["massFlowTable"].
    local aeroforceTable is profile["aeroforceTable"].

    function buildSegments {
        parameter steps.

        local segments is list().

        local stepIndex is 0.
        local startIndex is 0.
        local startVal is 0.

        until stepIndex + 1 >= steps:length {
            local stepSize is steps[stepIndex].
            local stepCount is ceiling((steps[stepIndex + 1] - startVal) / stepSize).
            local endVal is startVal + stepSize * stepCount.

            segments:add(list(startVal, endVal, startIndex, stepSize)).

            set startIndex to startIndex + stepCount.
            set startVal to endVal.

            set stepIndex to stepIndex + 2.
        }

        
        if stepIndex < steps:length {
            segments:add(list(startVal, 1e9, startIndex, steps[stepIndex])).
        }

        return segments.
    }

    local altitudeSegments is buildSegments(altitudeSteps).
    local speedSegments is buildSegments(speedSteps).

    function getPair {
        parameter value, segments, maxIndex.

        if value <= 0 {
            return list(0, 0, 0, 0).
        }

        local segLen is segments:length.

        local seg is segments[0].

        from { local i is 0. } until false step { set i to i + 1. } do {
            if i >= segLen - 1 or value < segments[i][1] {
                set seg to segments[i].
                break.
            }
        }

        local lowerIndex is min(
            seg[2] + floor((value - seg[0]) / seg[3]),
            maxIndex
        ).
        local upperIndex is min(
            lowerIndex + 1,
            maxIndex
        ).
        local lowerVal is seg[0] + (lowerIndex - seg[2]) * seg[3].
        local upperVal is lowerVal + seg[3].

        return list(lowerIndex, upperIndex, lowerVal, upperVal).
    }

    function getAltitudePair {
        parameter a.
        return getPair(a, altitudeSegments, ispTable:length - 1).
    }

    function getSpeedPair {
        parameter speed.
        return getPair(speed, speedSegments, aeroforceTable[0]:length - 1).
    }

    function interpolateAltitude {
        parameter table, a.

        if a <= 0 {
            return table[0].
        }

        local pair is getAltitudePair(a).
        local lowerIndex is pair[0].
        local upperIndex is pair[1].
        local lowerAltitude is pair[2].
        local upperAltitude is pair[3].

        if lowerIndex = upperIndex {
            return table[lowerIndex].
        }

        local fraction is (a - lowerAltitude) / (upperAltitude - lowerAltitude).

        return table[lowerIndex] + (table[upperIndex] - table[lowerIndex]) * fraction.
    }.

    function interpolateAeroforce {
        parameter a, speed.

        local altitudePair is getAltitudePair(a).

        local lowerAltitudeIndex is altitudePair[0].
        local upperAltitudeIndex is altitudePair[1].
        local lowerAltitude is altitudePair[2].
        local upperAltitude is altitudePair[3].

        local speedPair is getSpeedPair(speed).

        local lowerSpeedIndex is speedPair[0].
        local upperSpeedIndex is speedPair[1].
        local lowerSpeed is speedPair[2].
        local upperSpeed is speedPair[3].

        local lowerAltitudeLowerSpeed is aeroforceTable[lowerAltitudeIndex][lowerSpeedIndex].
        local lowerAltitudeUpperSpeed is aeroforceTable[lowerAltitudeIndex][upperSpeedIndex].

        local upperAltitudeLowerSpeed is aeroforceTable[upperAltitudeIndex][lowerSpeedIndex].
        local upperAltitudeUpperSpeed is aeroforceTable[upperAltitudeIndex][upperSpeedIndex].

        local speedFraction is 0.

        if upperSpeedIndex <> lowerSpeedIndex {
            set speedFraction to (speed - lowerSpeed) / (upperSpeed - lowerSpeed).
        }

        local lowerAltitudeValue is lowerAltitudeLowerSpeed
            + (lowerAltitudeUpperSpeed - lowerAltitudeLowerSpeed) * speedFraction.

        local upperAltitudeValue is upperAltitudeLowerSpeed
            + (upperAltitudeUpperSpeed - upperAltitudeLowerSpeed) * speedFraction.

        if upperAltitudeIndex = lowerAltitudeIndex {
            return lowerAltitudeValue.
        }

        local altitudeFraction is (a - lowerAltitude) / (upperAltitude - lowerAltitude).

        return lowerAltitudeValue
            + (upperAltitudeValue - lowerAltitudeValue) * altitudeFraction.
    }.

    return lex(
        "dryMass", dryMass,
        "wetMass", wetMass,
        "heightFromOriginToBottom", heightFromOriginToBottom,

        "ispat", {
            parameter a.
            return interpolateAltitude(ispTable, a).
        },

        "thrustat", {
            parameter a.
            return interpolateAltitude(thrustTable, a).
        },

        "massflowat", {
            parameter a.
            return interpolateAltitude(massFlowTable, a).
        },

        "aeroforceat", {
            parameter a, speed.
            return interpolateAeroforce(a, speed).
        }
    ).
}
