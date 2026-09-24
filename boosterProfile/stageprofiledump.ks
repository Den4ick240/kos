runoncepath(scriptpath():parent + "/getLiveProfile").
local procTag is core:tag.

if procTag = "" {
    set procTag to "test".
}

local liveProfile is getLiveProfile().

local speedSteps is list(
    25, 400,
    50, 1200,
    100
).

local altitudeSteps is list(
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

local dryMass is liveProfile:dryMass().
local wetMass is liveProfile:wetMass().
local massFlow is liveProfile:massflow.

if not gear {
    set gear to true.
    wait 5.
}

local heightFromOriginToBottom is liveProfile:heightFromOriginToBottom().

set gear to false.
wait 5.

local ispTable is list().
local thrustTable is list().
local retrogradeAeroforceTable is list().
local a is 0.
local aStepIndex is 0.
until a > body:atm:height {

    ispTable:add(liveProfile:ispat(a)).
    thrustTable:add(liveProfile:thrustat(a)).

    local aeroforceRow is list().
    local speedStepIndex is 0.

    local aeroforceSpeed is 0.
    until aeroforceSpeed > 1500 {
        aeroforceRow:add(liveProfile:retrogradeAeroforceat(a, aeroforceSpeed)).
        if  speedStepIndex + 2 < speedSteps:length and aeroforceSpeed >= speedSteps[speedStepIndex + 1] {
            set speedStepIndex to speedStepIndex + 2.
        }
        set aeroforceSpeed to aeroforceSpeed + speedSteps[speedStepIndex].
    }

    retrogradeAeroforceTable:add(aeroforceRow).

    if aStepIndex + 2 < altitudeSteps:length and a >= altitudeSteps[aStepIndex + 1] {
        set aStepIndex to aStepIndex + 2.
    }
    set a to a + altitudeSteps[aStepIndex].
}

set gear to true.

local stageProfile is lex(
    "altitudeSteps", altitudeSteps,
    "speedSteps", speedSteps,
    "dryMass", dryMass,
    "wetMass", wetMass,
    "heightFromOriginToBottom", heightFromOriginToBottom,
    "massFlow", massFlow,
    "ispTable", ispTable,
    "thrustTable", thrustTable,
    "retrogradeAeroforceTable", retrogradeAeroforceTable
).

writejson(stageProfile, jsonPath).
print "Wrote " + jsonPath.
