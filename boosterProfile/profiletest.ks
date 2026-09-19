runoncepath("0:/den4ick240kos/boosterProfile/loadProfile.ks").
local profilePath is "0:/bosterprofiles/test.json".
local profile is loadboosterprofile(profilePath).

local testAltitude is 15010.
local testSpeed is 1260.

print "=== PROFILE TEST ===".
print "Altitude: " + testAltitude.
print "Speed:    " + testSpeed.
print "".

local pressure is body:atm:altitudepressure(testAltitude).

local originalThrust is 0.
local originalMassFlow is 0.

for eng in ship:engines {
local thrust is eng:possiblethrustat(pressure).
local isp is eng:ispat(pressure).

set originalThrust to originalThrust + thrust.
set originalMassFlow to originalMassFlow + thrust / (isp * constant:g0).

}

local originalIsp is originalThrust / (originalMassFlow * constant:g0).
local originalAeroforce is addons:far:aeroforceat(
testAltitude,
-facing:forevector * testSpeed
):mag.

print "ISP".
print "  Original: " + originalIsp.
print "  Profile: " + profile:ispat(testAltitude).
print "".

print "Thrust".
print "  Original: " + originalThrust.
print "  Profile: " + profile:thrustat(testAltitude).
print "".

print "Mass flow".
print "  Original: " + originalMassFlow.
print "  Profile: " + profile:massflowat(testAltitude).
print "".

print "Aeroforce".
print "  Original: " + originalAeroforce.
print "  Profile: " + profile:aeroforceat(testAltitude, testSpeed).
print "".

print "Mass".
print "  Original dry: " + ship:drymass.
print "  Profile dry: " + profile:drymass().
print "  Original wet: " + ship:wetmass.
print "  Profile wet: " + profile:wetmass().
