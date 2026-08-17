// landingburn.ks - Landing burn with PID vertical velocity control
// Phase 1: full throttle until vertical speed is close to target
// Phase 2: PID outputs desired net vertical acceleration,
//           converted to throttle via mass, gravity, thrust, and attitude

// Phase 1: full brake, retrograde attitude until we approach target descent rate
lock throttle to 1.
lock steering to srfretrograde.
until ship:verticalspeed > -10 {
    wait 0.
}

// Phase 2: PID vertical velocity hold
local targetVertSpeed is -15.
local vertSpeedPID is PIDLoop(3, 0, 0, -50, 50).
set vertSpeedPID:setpoint to targetVertSpeed.
local desiredThrottle is 0.
lock throttle to desiredThrottle.

until ship:status = "LANDED" {
    if alt:radar < 80 {
        set targetVertSpeed to -5.
    } else {
        set targetVertSpeed to -55.
    }
    set vertSpeedPID:setpoint to targetVertSpeed.

    if ship:verticalspeed > targetVertSpeed - 1 {
        lock steering to up.
    } else {
        lock steering to srfretrograde.
    }

    local bodyPos is ship:body:position.
    local g is ship:body:mu / (ship:position - bodyPos):sqrmagnitude.
    local facingDotUp is vdot(ship:facing:forevector, ship:up:vector).

    local pressure is 0.
    if ship:body:atm:exists {
        set pressure to ship:body:atm:altitudepressure(ship:altitude).
    }
    local maxThrust_ is ship:availablethrustat(pressure).

    // PID: error = setpoint - verticalspeed
    local desiredAccel is vertSpeedPID:update(time:seconds, ship:verticalspeed).

    // thr = mass * (desiredAccel + g) / (thrust * facingDotUp)
    if facingDotUp > 0.01 and maxThrust_ > 0 {
        set desiredThrottle to max(0, min(1,
            (ship:mass * (desiredAccel + g)) / (maxThrust_ * facingDotUp)
        )).
    } else {
        set desiredThrottle to 0.
    }

    wait 0.
}

set desiredThrottle to 0.
unlock steering.
print "landed".
