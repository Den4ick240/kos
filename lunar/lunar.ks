runoncepath(scriptpath():parent + "/integrateLowTwrLanding").
runoncepath(scriptpath():parent:parent + "/getSelectedWaypoint").
runoncepath(scriptpath():parent + "/getStageProfiles").
runoncepath(scriptpath():parent:parent + "/apdg").

function trueAnomalyToMeanAnomaly {
    parameter trueAnomaly, o.
    local e is o:eccentricity.
    local ea is arctan2(sqrt(1 - e ^ 2) * sin(trueAnomaly), e + cos(trueAnomaly)).
    if ea < 0 {
        set ea to ea + 360.
    }
    local meanAnomaly is ea - e * sin(ea) * constant:radtodeg.
    if meanAnomaly < 0 {
        return meanAnomaly + 360.
    } else {
        return meanAnomaly.
    }
}

function getMeanMotion {
    parameter o.
    return sqrt(o:body:mu / o:semimajoraxis ^ 3) * constant:radtodeg.
}

function signedVectorAngle {
    parameter a, b, axis.
    return arctan2(axis * vcrs(a, b), a * b).
}

function runLunar {

local waypoint_ is getSelectedWaypoint().
local landingSite is waypoint_:geoposition.
local targetAltitude is landingSite:terrainheight.

local stage1Profile is getStageProfile(1).
local stage2Profile is getStageProfile(2).

local profileAfterManuver is stage2Profile:copy().

for n in allnodes { remove n. }
wait 0.

local inclinationNode is node(0, 0, 0, 0).
local retrogradeNode is node(0, 0, 0, 0).
local landingNode is node(0, 0, 0, 0).

add inclinationNode.
add retrogradeNode.
add landingNode.

local approachAltitude is ship:altitude.
local previousTime is time:seconds.
local virtualTargetVector is landingSite:position - ship:body:position.
local crossRangeError is 0.
local downRangeError is 0.

local burnTime is 0.
local landingTime is 0.

wait 0. // make sure all calculations up to integration happen in one frame.
until false {
    local dt is time:seconds - previousTime.
    set previousTime to time:seconds.
    local targetVector is landingSite:position - ship:body:position.
    local selfVector is ship:position - ship:body:position.

    local orbitNormal is vcrs(selfVector, ship:velocity:orbit):normalized.

    local selfDir is selfVector:normalized.
    local inclinationNodeDir is vcrs(virtualTargetVector, orbitNormal):normalized.

    local targetOrbitNormal is vcrs(virtualTargetVector, inclinationNodeDir):normalized.
    if vdot(targetOrbitNormal, orbitNormal) < 0 {
        set targetOrbitNormal to targetOrbitNormal * -1.
        set inclinationNodeDir to inclinationNodeDir * -1.
    }

    local trueanomaly is ship:orbit:trueanomaly.
    local inclinationNodeTrueAnomaly is trueAnomaly + 
        arctan2(orbitNormal * vcrs(selfDir, inclinationNodeDir), selfDir * inclinationNodeDir).

    local meanAnomaly is trueAnomalyToMeanAnomaly(trueAnomaly, ship:orbit).
    local inclinationNodeMeanAnomaly is trueAnomalyToMeanAnomaly(inclinationNodeTrueAnomaly, ship:orbit).
    local meanMotion is getMeanMotion(ship:orbit).

    if inclinationNodeMeanAnomaly < meanAnomaly {
        set inclinationNodeMeanAnomaly to inclinationNodeMeanAnomaly + 360.
    }

    local inclinationNodeEta is (inclinationNodeMeanAnomaly - meanAnomaly) / meanMotion.
    set inclinationNode:eta to inclinationNodeEta.

    local deltaInclination is vectorangle(targetOrbitNormal, orbitNormal).
    local planeChangeAxis is inclinationNodeDir.
    local currentInclinationNodeVelocity is velocityat(ship, time:seconds + inclinationNodeEta):orbit.
    local targetInclinationNodeVelocity is angleaxis(deltaInclination, planeChangeAxis:normalized) * currentInclinationNodeVelocity.
    local inclinationDv is targetInclinationNodeVelocity - currentInclinationNodeVelocity.

    set inclinationNode:prograde to inclinationDv * currentInclinationNodeVelocity:normalized.
    set inclinationNode:normal to inclinationDv * orbitNormal.

    local retrogradeNodeTrueAnomaly is inclinationNodeTrueAnomaly + 90.
    local retrogradeNodeMeanAnomaly is trueAnomalyToMeanAnomaly(retrogradeNodeTrueAnomaly, ship:orbit).
    until retrogradeNodeMeanAnomaly > inclinationNodeMeanAnomaly {
        set retrogradeNodeMeanAnomaly to retrogradeNodeMeanAnomaly + 360.
    }
    local retrogradeNodeEta is inclinationNodeEta + (retrogradeNodeMeanAnomaly - inclinationNodeMeanAnomaly) / meanMotion.
    local retrogradeNodeTime is retrogradeNodeEta + time:seconds.

    set retrogradeNode:eta to retrogradeNodeEta.
    local retrogradeNodePosition is positionat(ship, retrogradeNodeTime) - body:position.
    local retrogradeNodeVelocity is velocityat(ship, retrogradeNodeTime - 0.01):orbit.
    local retrogradeNodeDir is retrogradeNodePosition:normalized.
    local retrogradeTargetVelocityDir is vectorexclude(retrogradeNodeDir, retrogradeNodeVelocity):normalized.
    local approachR is approachAltitude + ship:body:radius.
    local retrogradeNodeR is retrogradeNodePosition:mag.
    local retrogradeTargetVelocity is retrogradeTargetVelocityDir * sqrt(
        2 * ship:body:mu * approachR / (retrogradeNodeR * (retrogradeNodeR + approachR))
    ).
    local retrogradeDv is retrogradeTargetVelocity - retrogradeNodeVelocity.

    set retrogradeNode:prograde to retrogradeDv * retrogradeNodeVelocity:normalized.
    set retrogradeNode:radialout to vectorexclude(retrogradeNodeVelocity, retrogradeDv):mag. // retrogradeDv should only have prograde and radialout

    local approachTime is retrogradeNodeTime + 0.5 * retrogradeNode:orbit:period.

    local totalManuverDv is inclinationDv:mag + retrogradeDv:mag.
    set profileAfterManuver:mass to stage2Profile:mass * constant:e ^ (
        -totalManuverDv / (stage2Profile:isp * constant:g0)
    ).

    local approachCircularizationDv is max(0, velocityat(ship, approachTime):orbit:mag - sqrt(ship:body:mu / approachR)).
    local approachCircularizationTime is (profileAfterManuver:mass / profileAfterManuver:massflow) * (
        1 - constant:e ^ (-approachCircularizationDv / (profileAfterManuver:isp * constant:g0))
    ).

    local burnStartTime is approachTime - approachCircularizationTime.

    set landingNode:eta to burnStartTime - time:seconds.

    local burnStartVelocity is velocityat(ship, burnStartTime):orbit.

    local landing1 is integrateLowTwrLanding( // this will probably take multiple frames
        lex(
            "body", ship:body,
            "position", positionat(ship, burnStartTime) - ship:body:position,
            "velocity", burnStartVelocity,
            "mass", profileAfterManuver:mass,
            "targetAltitude", targetAltitude,
            "massflow", profileAfterManuver:massflow,
            "thrust", profileAfterManuver:thrust,
            "dryMass", profileAfterManuver:dryMass
        )
    ).

    local landing is integrateLowTwrLanding(
        lex(
            "body", ship:body,
            "position", landing1:position,
            "velocity", landing1:velocity,
            "mass", stage1Profile:mass,
            "targetAltitude", targetAltitude,
            "massflow", stage1Profile:massflow,
            "thrust", stage1Profile:thrust,
            "dryMass", stage1Profile:dryMass
        )
    ).

    local landedTime is burnStartTime + landing1:time + landing:time.

    local altitudeError is targetAltitude - (landing:position:mag - ship:body:radius).

    set approachAltitude to approachAltitude + altitudeError * 0.5.

    wait 0. // make sure from here and up to next integration calculations happen in one frame
    set predictedTargetVector to angleaxis(
        ship:body:angularvel:mag * constant:radtodeg * (landedTime - time:seconds), 
        ship:body:angularvel
    ) * (landingSite:position - ship:body:position).

    local downRangeAxis is targetOrbitNormal.
    local landingDir is landing:position:normalized.
    local newDownRangeError is signedVectorAngle(landingDir, predictedTargetVector:normalized, downRangeAxis).
    set downRangeError to downRangeError + 0.5 * newDownRangeError.
    set virtualTargetVector to angleaxis(downRangeError, downRangeAxis) * predictedTargetVector.

    local crossRangeAxis is vcrs(predictedTargetVector, downRangeAxis):normalized.
    local newCrossRangeError is signedVectorAngle(landingDir, predictedTargetVector, crossRangeAxis).
    set crossRangeError to crossRangeError + 0.5 * newCrossRangeError .
    set virtualTargetVector to angleaxis(crossRangeError, crossRangeAxis) * virtualTargetVector.

    //print (landing:position - predictedTargetVector):mag.
    print newDownRangeError.
    if (landing:position - predictedTargetVector):mag < 20 {
        set burnTime to landing1:time + landing:time.
        set landingTime to landedTime.
        break.
    }
}

print "waiting for eta".
wait until landingNode:eta < 0.5.
sas on.
wait 0.
set sasmode to "RETROGRADE".
lock throttle to 0.9.
wait until (landingTime - time:seconds) < 60.
sas off.
unlock throttle.

runApdg(landingSite,  landingTime).

}

runLunar().
