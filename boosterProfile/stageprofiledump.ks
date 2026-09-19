local procTag is core:tag.

if procTag = "" {
    set procTag to "test".
}

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

local dryMass is ship:drymass.
local wetMass is ship:wetmass.

if not gear {
    set gear to true.
    wait 5.
}

local heightFromOriginToBottom is vdot(-facing:forevector, ship:bounds:furthestcorner(facing:forevector) - ship:position).

set gear to false.
wait 5.

local usedEngines is ship:engines.

local ispTable is list().
local thrustTable is list().
local massFlowTable is list().
local aeroforceTable is list().
local a is 0.
local aStepIndex is 0.
until a > body:atm:height {
    local pressure is body:atm:altitudepressure(a).
    local totalThrust is 0.
    local totalMassFlow is 0.
    for eng in usedEngines {
       local thrust is eng:possiblethrustat(pressure). 
       local isp is eng:ispat(pressure).
       set totalThrust to totalThrust + thrust.
       set totalMassFlow to totalMassFlow + thrust / (isp * constant:g0).
    }
    local totalIsp is totalThrust / (totalMassFlow * constant:g0).

    ispTable:add(totalIsp).
    thrustTable:add(totalThrust).
    massFlowTable:add(totalMassFlow).

    local aeroforceRow is list().
    local speedStepIndex is 0.

    local aeroforceSpeed is 0.
    until aeroforceSpeed > 1500 {
        aeroforceRow:add(round(addons:far:aeroforceat(a, -facing:forevector * aeroforceSpeed):mag, 4)).
        if  speedStepIndex + 2 < speedSteps:length and aeroforceSpeed >= speedSteps[speedStepIndex + 1] {
            set speedStepIndex to speedStepIndex + 2.
        }
        set aeroforceSpeed to aeroforceSpeed + speedSteps[speedStepIndex].
    }

    aeroforceTable:add(aeroforceRow).

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
    "ispTable", ispTable,
    "thrustTable", thrustTable,
    "massFlowTable", massFlowTable,
    "aeroforceTable", aeroforceTable
).

writejson(stageProfile, jsonPath).
print "Wrote " + jsonPath.
