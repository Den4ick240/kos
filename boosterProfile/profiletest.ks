runoncepath(scriptpath():parent + "/loadProfile").
runoncepath(scriptpath():parent + "/getLiveProfile").
local profilePath is "0:/bosterprofiles/test.json".
local profile is loadboosterprofile(profilePath).
local liveProfile is getLiveProfile().

local testAltitude is 1510.
local testSpeed is 1260.

print "=== PROFILE TEST ===".
print "Altitude: " + testAltitude.
print "Speed:    " + testSpeed.
print "".

print "ISP".
print "  Original: " + liveProfile:ispat(testAltitude).
print "  Profile: " + profile:ispat(testAltitude).
print "".

print "Thrust".
print "  Original: " + liveProfile:thrustat(testAltitude).
print "  Profile: " + profile:thrustat(testAltitude).
print "".

print "Mass flow".
print "  Original: " + liveProfile:massflow().
print "  Profile: " + profile:massflow().
print "".

print "Aeroforce".
print "  Original: " + liveProfile:retrogradeAeroforceat(testAltitude, testSpeed).
print "  Profile: " + profile:retrogradeAeroforceat(testAltitude, testSpeed).
print "".

print "Mass".
print "  Original dry: " + liveProfile:drymass().
print "  Profile dry: " + profile:drymass().
print "  Original wet: " + liveProfile:wetmass().
print "  Profile wet: " + profile:wetmass().
