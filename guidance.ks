// ============================================================================
// GUIDANCE
// ============================================================================
// Predicts where the current trajectory impacts, measures the horizontal error
// to the landing site, and steers the retrograde direction onto it, producing
// desiredDir (a Direction) for the attitude controller to hold.
//
// Public API:
//   guidanceInit(landingSite)       - one-time setup
//   guidanceUpdate() -> Vector      - one guidance step, called per frame
//
// Tuning (globals, override after runoncepath):
//   gKp, gKd, gSteerSign, gMaxSteer, gErrorFilter
//
// Diagnostics (globals):
//   guidDnErr, guidCrErr (m), guidSteerAngle (deg), guidDnOut, guidCrOut (deg)

set gKp to 0.2.         // deg of steer per meter of horizontal error
set gKd to 0.02.         // deg per (m/s) of error closing rate
set gSteerSign to 1.0.   // set to -1 if the aero force pushes the wrong way
set gMaxSteer to 60.     // PID output clamp (deg off retrograde per axis)
set gErrorFilter to 0.6. // 0..1 lowpass on the measured error

local guidDownrangePID is PIDLoop(gKp, 0.01, gKd, -gMaxSteer, gMaxSteer).
local guidCrossrangePID is PIDLoop(gKp, 0, gKd, -gMaxSteer, gMaxSteer).
set guidDownrangePID:setpoint to 0.
set guidCrossrangePID:setpoint to 0.

local guidFilteredErr is v(0,0,0).
local guidSite to ship:position. // placeholder until guidanceInit is called

local guidDnErr to 0.
local guidCrErr to 0.
local guidSteerAngle to 0.
local guidDnOut to 0.
local guidCrOut to 0.

function guidanceInit {
    parameter site.
    set guidSite to site.
    // Re-init PID state in case this file was already loaded for a prior launch.
    guidDownrangePID:reset().
    guidCrossrangePID:reset().
    set guidFilteredErr to v(0,0,0).
}

function guidanceUpdate {
    // 1. Integrate the trajectory to predict where we'd hit right now.
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        80000, 0.02, 8,
        "rk4", 0.1, 0.001,
        guidSite:terrainheight
    ).
    local hitGeo is hitData["impactGeo"].
    addons:tr:settarget(hitgeo).

    // 2. Error: horizontal vector from the impact point toward the landing site.
    local errVec is vectorExclude(ship:up:vector, guidSite:position - hitGeo:position).

    set guidFilteredErr to guidFilteredErr + (errVec - guidFilteredErr) * gErrorFilter.

    // Downrange axis: where the surface velocity is pointing horizontally.
    // Falling straight down, fall back to the error direction.
    local downrangeDir is vectorExclude(ship:up:vector, ship:velocity:surface).
    if downrangeDir:mag > 0.001 {
        set downrangeDir to downrangeDir:normalized.
    } else if errVec:mag > 0.001 {
        set downrangeDir to errVec:normalized.
    } else {
        set downrangeDir to ship:facing:forevector.
    }
    local crossrangeDir is vcrs(ship:up:vector, downrangeDir).
    if crossrangeDir:mag > 0.001 {
        set crossrangeDir to crossrangeDir:normalized.
    } else {
        set crossrangeDir to ship:facing:starvector.
    }

    set guidDnErr to vdot(guidFilteredErr, downrangeDir).
    set guidCrErr to vdot(guidFilteredErr, crossrangeDir).

    // 3. Built-in PIDs, one per axis, output steer offsets in degrees.
    set guidDnOut to guidDownrangePID:update(time:seconds, guidDnErr).
    set guidCrOut to guidCrossrangePID:update(time:seconds, guidCrErr).

    local ctrlVec is downrangeDir * guidDnOut + crossrangeDir * guidCrOut.
    set guidSteerAngle to ctrlVec:mag.
    if guidSteerAngle > gMaxSteer { set guidSteerAngle to gMaxSteer. }

    set anArrow1 to vecdraw(
        {return ship:position.},
        guidSite:position - hitGeo:position,
        rgb(0,1,0),
        "err",
        1.0, true, 0.2, true, true
    ).

    // 4. Steer: rotate the retrograde direction toward the error by steerAngle.
    local desiredDir is srfretrograde:forevector.
    if guidSteerAngle > 0.001 {
        local retUnit is -ship:velocity:surface:normalized.
        local axis is vcrs(retUnit, ctrlVec):normalized.
        if axis:mag > 0.001 {
            set desiredDir to angleaxis(guidSteerAngle * gSteerSign, axis) * retUnit.
        }
    }

    set anArrow2 to vecdraw(
        {return ship:position.},
        desiredDir * 200,
        rgb(1,0,0),
        "steer",
        1.0, true, 0.2, true, true
    ).

    print "guid dn " + round(guidDnErr, 0) + "m cr " + round(guidCrErr, 0) + "m steer " + round(guidSteerAngle, 1) + "deg pid " + round(guidDnOut, 1) + "/" + round(guidCrOut, 1) at (0, 22).
    return desiredDir.
}
