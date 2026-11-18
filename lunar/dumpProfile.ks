runoncepath(scriptpath():parent:parent + "boosterProfile/interpolateTable").
runoncepath(scriptpath():parent:parent + "profile/name").

parameter stageNumber is 1.  

local suffix is "".
if stageNumber != 1 {
    set suffix to "stage_" + stageNumber.
}
local profilePath profileName("lander", suffix).




