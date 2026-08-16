# Landing Burn Simulation in integrateTrajectory

## File: boosterlib.ks

### 1. Global `burnMassFlowRate` — once per physics tick

Add before `integrateTrajectory` (after `displayPredictedHit`):

```
global burnMassFlowRate is 0.
when true then {
    list engines in _engList.
    set burnMassFlowRate to 0.
    for eng in _engList {
        if eng:ignition {
            set burnMassFlowRate to burnMassFlowRate + eng:maxmassflow.
        }
    }
    return true.
}
```

### 2. Inside `integrateTrajectory` — new locals (after line ~136)

```
local burnAlt is 2500.
local currentMass is shipMass.
local inBurn is false.
```

### 3. Modify `getForce` (line ~148)

- **Line 161**: `shipMass` → `currentMass`
- **After line 161, before `local acc`**: add thrust block:

```
local thrustAcc is v(0,0,0).
local simAlt is position:mag - bodyRadius.
if not inBurn and burnMassFlowRate > 0 and (simAlt - targetAltitude) <= burnAlt {
    set inBurn to true.
}
if inBurn {
    local vSurf is vel - vcrs(omega, position).
    if vSurf:mag > 0.5 {
        local pressure is 0.
        if body_:atm:exists {
            set pressure to body_:atm:altitudepressure(simAlt).
        }
        local curThrust is ship:availablethrustat(pressure).
        set thrustAcc to -vSurf:normalized * (curThrust / currentMass).
    }
}
```

- **Line 163**: `local acc is gAcc + aeroAcc.` → `local acc is gAcc + aeroAcc + thrustAcc.`

### 4. Mass tracking + hover exit — after `set steps to steps + 1` (line ~218)

```
if inBurn {
    set currentMass to currentMass - burnMassFlowRate * dt.
    if currentMass < 1 { set currentMass to 1. }
    local vSurf is vel - vcrs(omega, position).
    local vertSpeed is vdot(vSurf, position:normalized).
    if vSurf:mag < 0.5 or vertSpeed > 0 {
        break.
    }
}
```

### 5. Post-loop — hover-down projection (replace lines ~221–225)

```
if altitude_ > targetAltitude {
    set position to position:normalized * (targetAltitude + bodyRadius).
} else {
    local prevHeightAboveTarget is prevAltitude - targetAltitude.
    local curHeightAboveTarget is altitude_ - targetAltitude.
    local frac is prevHeightAboveTarget / (prevHeightAboveTarget - curHeightAboveTarget).
    set position to prevPosition + (position - prevPosition) * frac.
    set hitTime to prevHitTime + (hitTime - prevHitTime) * frac.
}
```

### 6. Update callers (minor)

`displayPredictedHit` and other callers pass `targetAltitude` as 2nd positional param now (already matches). No changes needed to callers.
