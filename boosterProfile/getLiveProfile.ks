function getHeightFromOriginToBottom {
    return vdot(-facing:forevector, ship:bounds:furthestcorner(facing:forevector) - ship:position).
}

function getMassFlow {
    parameter engines.
    local totalMassFlow is 0.
    for eng in engines {
        set totalMassFlow to totalMassFlow + eng:maxmassflow * eng:thrustLimit / 100.
    }
    return totalMassFlow.
}

function getLiveProfile {
    parameter engines is ship:engines.

    local liveStateActive is true.
    local heightFromOriginToBottom is getHeightFromOriginToBottom().
    local massFlow is getMassFlow(engines).

    local nextUpdateTime is time:seconds + 5.
    when nextUpdateTime < time:seconds then {
        if not liveStateActive {
            return false.
        }
        set heightFromOriginToBottom to getHeightFromOriginToBottom().
        set massFlow to getMassFlow(engines).
        set nextUpdateTime to time:seconds + 5.
        return liveStateActive.
    }

    return lex(
        "dryMass", { return ship:drymass. },
        "wetMass", { return ship:wetmass. },
        "heightFromOriginToBottom", {
            return heightFromOriginToBottom.
        } ,

        "ispat", {
            parameter a.

            local pressure is body:atm:altitudepressure(a).
            local totalThrust is 0.
            local totalMassFlow is 0.
            for eng in engines {
                local thrustLimit is eng:thrustLimit / 100.
                set totalThrust to totalThrust + eng:possiblethrustat(pressure) * thrustLimit.
                set totalMassFlow to totalMassFlow + eng:maxmassflow * thrustLimit.
            }
            return totalThrust / (totalMassFlow * constant:g0).
        },

        "thrustat", {
            parameter a.

            local pressure is body:atm:altitudepressure(a).
            local totalThrust is 0.
            for eng in engines {
                set totalThrust to totalThrust + eng:possiblethrustat(pressure) * eng:thrustLimit / 100.
            }
            return totalThrust. 
        },

        "massflow", {
            return massFlow.
        },

        "retrogradeAeroforceat", {
            parameter a, speed.
            return addons:far:aeroforceat(a, -facing:forevector * speed):mag.
        },

        "close", {
            set liveStateActive to false.
        }
    ).
}
