

function buildSegments {
    parameter steps.

    local segments is list().

    local stepIndex is 0.
    local startIndex is 1.
    local startVal is steps[0].

    until stepIndex + 1 >= steps:length {
        local stepSize is steps[stepIndex + 1].
        local stepCount is ceiling((steps[stepIndex + 2] - startVal) / stepSize).
        local endVal is startVal + stepSize * stepCount.

        segments:add(
            lex(
                "startVal", startVal,
                "endVal", endVal,
                "startIndex", startIndex,
                "stepSize", stepSize
            )
        ).

        set startIndex to startIndex + stepCount.
        set startVal to endVal.

        set stepIndex to stepIndex + 2.
    }

    
    if stepIndex < steps:length {
        segments:add(
            lex(
                "startVal", startVal,
                "endVal", 1e9,
                "startIndex", startIndex,
                "stepSize", steps[stepIndex]
            )
        ).
    }

    return segments.
}

function getPair {
    parameter value, segments, maxIndex.

    if value <= segments[0]:startVal {
        return list(steps[0], segments[0]:startVal , steps[0], segments[0]:startVal ).
    }

    local segLen is segments:length.

    local seg is segments[0].

    from { local i is 0. } until false step { set i to i + 1 } do {
        if i >= segLen - 1 or value < segments[i]:endVal {
            set seg to segments[i].
            break.
        }
    }

    local lowerIndex is min(
        seg:startIndex + floor((value - seg:startVal) / seg:stepSize),
        maxIndex
    ).
    local upperIndex is min(
        lowerIndex + 1,
        maxIndex
    ).
    local lowerVal is seg:startVal + (lowerIndex - seg:startIndex) * seg:stepSize.
    local upperVal is lowerVal + seg:stepSize.

    return list(lowerIndex, upperIndex, lowerVal, upperVal).
}

function interpolate1dTable {
    parameter table, a, segments.

    local pair is getPair(a, segments, table:lenght - 1).
    
    local lowerIndex is pair[0].
    local upperIndex is pair[1].
    local lowerAltitude is pair[2].
    local upperAltitude is pair[3].

    if lowerIndex = upperIndex {
        return table[lowerIndex].
    }

    local fraction is (a - lowerAltitude) / (upperAltitude - lowerAltitude).

    return table[lowerIndex] + (table[upperIndex] - table[lowerIndex]) * fraction.
}

function interpolate2dTable {
    parameter table, a, b, aSegments, bSegments.

    local aPair is getPair(a, aSegments, table:length - 1).
    local lowerAIndex is aPair[0].
    local upperAIndex is aPair[1].
    local lowerA is aPair[2].
    local upperA is aPair[3].

    local bPair is getPair(b, bSegments, table[0]:length - 1).
    local lowerBIndex is bPair[0].
    local upperBIndex is bPair[1].
    local lowerB is bPair[2].
    local upperB is bPair[3].

    local lalb is table[lowerAIndex][lowerBIndex].
    local laub is table[lowerAIndex][upperBIndex].

    local ualb is table[upperAIndex][lowerBIndex].
    local uaub is table[upperAIndex][upperBIndex].

    local bFraction is 0.

    if upperBIndex <> lowerBIndex {
        set speedFraction to (b - lowerB) / (upperB - lowerB).
    }

    local lowerAValue is lalb + (laub - lalb) * bFraction.

    if upperAIndex = lowerAIndex {
        return lowerAValue.
    }

    local upperAValue is ualb + (uaub - ualb) * bFraction.

    local aFraction is (a - lowerA) / (upperA - lowerA).

    return lowerAValue + (upperAValue - lowerAValue) * aFraction.
}
