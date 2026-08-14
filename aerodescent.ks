runoncepath("0:/den4ick240kos/boosterlib.ks").

// ============================================================================
// ATMOSPHERIC DESCENT GUIDANCE
// ============================================================================
// Loop per frame:
//   1. INTEGRATE  - predict the impact point of the current trajectory.
//   2. ERROR      - split horizontal error into downrange / crossrange axes.
//   3. STEER      - a built-in PIDLoop per axis outputs a steer offset
//                   (degrees); kOS clamps each output to +/-maxSteer.
//                   The combined vector rotates the retrograde direction onto
//                   the error so the airbrakes/aero surfaces push us to target.

parameter landingSite.
parameter kp to 0.04.        // deg of steer per meter of horizontal error
parameter kd to 0.02.        // deg per (m/s) of error closing rate, damps oscillation
parameter steerSign to 1.0.  // set to -1 if the aero force pushes the wrong way
parameter maxSteer to 25.    // PID output clamp: max degrees off retrograde per axis
parameter errorFilter to 0.3. // 0..1 lowpass on the measured error

// ---- Steering manager tuning for thick-atmosphere descent ----
// kOS computes available torque with the STOCK torque calculation, which does
// not include FAR control-surface authority. With airbrakes deployed the drag
// torque is huge, so the manager under-commands and can't hold the desired
// direction. These tell it to assume more torque (factor), allow faster
// rotation (maxStoppingTime) and anticipate stopping earlier (steerKD).
parameter maxStoppingTime to 8.0.   // seconds; higher = faster allowed turn rate
parameter torqueFactor to 3.0.      // multiplier on assumed available pitch/yaw torque
parameter steerKP to 1.0.           // rotational-velocity PID proportional gain
parameter steerKI to 0.1.           // rotational-velocity PID integral gain
parameter steerKD to 1.5.           // rotational-velocity PID derivative gain

lock throttle to 0.

// Reset to a known baseline, then apply the descent tuning.
steeringmanager:resettodefault().
set steeringmanager:maxstoppingtime to maxStoppingTime.
set steeringmanager:pitchtorquefactor to torqueFactor.
set steeringmanager:yawtorquefactor to torqueFactor.
set steeringmanager:pitchpid:kp to steerKP.
set steeringmanager:pitchpid:ki to steerKI.
set steeringmanager:pitchpid:kd to steerKD.
set steeringmanager:yawpid:kp to steerKP.
set steeringmanager:yawpid:ki to steerKI.
set steeringmanager:yawpid:kd to steerKD.
print "steering tuned: maxstop " + maxStoppingTime + "s torqueX " + torqueFactor + " PID kd " + steerKD at (0, 19).

local downrangePID is PIDLoop(kp, 0, kd, -maxSteer, maxSteer).
local crossrangePID is PIDLoop(kp, 0, kd, -maxSteer, maxSteer).
set downrangePID:setpoint to 0.
set crossrangePID:setpoint to 0.

local filteredErr is v(0,0,0).

until false {
    // 1. Integrate the trajectory to predict where we'd hit right now.
    local hitData is integrateTrajectory(
        ship:velocity:orbit,
        80000, 0.1, 8,
        "rk45", 0.1, 0.001
    ).
    local hitGeo is hitData["impactGeo"].
    addons:tr:settarget(hitGeo).

    // 2. Error: horizontal vector from the impact point toward the landing site.
    local errVec is vectorExclude(ship:up:vector, landingSite:position - hitGeo:position).
    local errMag is errVec:mag.

    set filteredErr to filteredErr + (errVec - filteredErr) * errorFilter.

    // Downrange axis: where the surface velocity is pointing horizontally.
    // Falling straight down, fall back to the error direction.
    local downrangeDir is vectorExclude(ship:up:vector, ship:velocity:surface).
    if downrangeDir:mag > 0.001 {
        set downrangeDir to downrangeDir:normalized.
    } else {
        set downrangeDir to errVec:normalized.
    }
    local crossrangeDir is vcrs(ship:up:vector, downrangeDir):normalized.

    local dnErr is vdot(filteredErr, downrangeDir).
    local crErr is vdot(filteredErr, crossrangeDir).

    // 3. Built-in PIDs, one per axis, output steer offsets in degrees.
    local dnOut is downrangePID:update(time:seconds, dnErr).
    local crOut is crossrangePID:update(time:seconds, crErr).

    local ctrlVec is downrangeDir * dnOut + crossrangeDir * crOut.
    local steerAngle is ctrlVec:mag.
    if steerAngle > maxSteer { set steerAngle to maxSteer. }

    set anArrow1 to vecdraw(
        {return ship:position.},
        landingSite:position - hitGeo:position,
        rgb(0,1,0),
        "err",
        1.0, true, 0.2, true, true
    ).

    // 4. Steer: rotate the retrograde direction toward the error by steerAngle.
    local desiredDir is srfretrograde:forevector.
    if steerAngle > 0.001 {
        local retUnit is -ship:velocity:surface:normalized.
        local axis is vcrs(retUnit, ctrlVec):normalized.
        if axis:mag > 0.001 {
            set desiredDir to angleaxis(steerAngle * steerSign, axis) * retUnit.
        }
    }
    lock steering to lookdirup(desiredDir, ship:up:vector).

    set anArrow2 to vecdraw(
        {return ship:position.},
        desiredDir * 200,
        rgb(1,0,0),
        "steer",
        1.0, true, 0.2, true, true
    ).

    print "aero dn " + round(dnErr, 0) + "m cr " + round(crErr, 0) + "m steer " + round(steerAngle, 1) + "deg pid " + round(dnOut, 1) + "/" + round(crOut, 1) at (0, 22).
    print "sm err " + round(steeringmanager:anglerror, 1) + "deg p " + round(steeringmanager:pitcherror, 1) + " y " + round(steeringmanager:yawerror, 1) at (0, 23).

    wait 0.
}
