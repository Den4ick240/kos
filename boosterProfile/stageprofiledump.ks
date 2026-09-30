runoncepath(scriptpath():parent + "/getLiveProfile").
runoncepath(scriptpath():parent + "/buildTable").
local procTag is core:tag.

if procTag = "" {
    set procTag to "test".
}

local liveProfile is getLiveProfile().

local speedSteps is list(
    0,
    25, 400,
    50, 1200,
    100
).

local altitudeSteps is list(
    0,
    250, 1000,
    1000, 10000,
    2500, 25000,
    5000
).

print "CPU part tag: " + procTag.

local jsonPath is "0:/bosterprofiles/" + procTag + ".json".

if exists(jsonPath) {
    deletepath(jsonPath).
}

if not gear {
    set gear to true.
    wait 5.
}

local heightFromOriginToBottom is liveProfile:heightFromOriginToBottom().

set gear to false.
wait 5.

local stageProfile is lex(
    "altitudeSteps", altitudeSteps,
    "speedSteps", speedSteps,
    "dryMass", liveProfile:dryMass(),
    "wetMass", liveProfile:wetMass(),
    "heightFromOriginToBottom", heightFromOriginToBottom,
    "massFlow", liveProfile:massflow,
    "ispTable", build1dTable(altitudeSteps, liveProfile:ispat@),
    "thrustTable", build1dTable(altitudeSteps, liveProfile:thrustat@),
    "retrogradeAeroforceTable", build2dTable(altitudeSteps, speedSteps, liveProfile:aeroforceSpeed@)
).

set gear to true.

writejson(stageProfile, jsonPath).
print "Wrote " + jsonPath.
