set surfModules to list().
set surfAutoFlip to true.
set surfDeadband to 2.
set surfVerbose to true.

set surfSignLast to 1.
set surfFlips to 0.

function surfaceDeflectionFieldName {
    parameter m.
    if m:hasfield("Ctrl Dflct") {
        return "Ctrl Dflct".
    }
    for f in m:allfieldnames {
        if f:contains("Dflct") {
            return f.
        }
    }
    return "".
}

function surfaceScan {
    set surfModules to list().
    local allParts is list().
    list parts in allParts.
    for p in allParts {
        if p:hasmodule("ModuleControlSurface") or p:hasmodule("FARControllableSurface") {
            local m is p:getmodule("FARControllableSurface").
            if not p:hasmodule("FARControllableSurface") {
                set m to p:getmodule("ModuleControlSurface").
            }
            surfModules:add(m).
        }
    }
    print "surfaces scanned: " + surfModules:length at (0, 27).
}

function surfaceFwdVel {
    return vdot(ship:velocity:surface, ship:facing:forevector).
}

function surfaceSignNow {
    local fwd is surfaceFwdVel().
    if fwd > surfDeadband {
        set surfSignLast to 1.
    } else if fwd < -surfDeadband {
        set surfSignLast to -1.
    }
    return surfSignLast.
}

function surfaceFlipCtrlDflct {
    local n is 0.
    for m in surfModules {
        local f is surfaceDeflectionFieldName(m).
        if f <> "" {
            m:setfield(f, -m:getfield(f)).
            set n to n + 1.
        }
    }
    if surfVerbose {
        print "ctrl dflct flipped x" + n at (0, 28).
    }
    return n.
}

function surfaceFlipTick {
    set surfFlips to surfFlips + 1.
    if surfAutoFlip {
        surfaceFlipCtrlDflct().
    }
    if surfVerbose {
        print "surf dir " + surfSignLast + " flips " + surfFlips at (0, 27).
    }
}

function surfaceFlipInit {
    if surfModules:length = 0 {
        surfaceScan().
    }
    set surfSignLast to surfaceSignNow().
    on surfaceSignNow() {
        surfaceFlipTick().
        return true.
    }
    print "surface flip armed" at (0, 27).
}
