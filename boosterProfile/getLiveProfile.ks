
function getLiveProfile {
    parameter engines is ship:engines.
    return lex(
        "dryMass", { return ship:drymass. },
        "wetMass", { return ship:wetmass. },
        "heightFromOriginToBottom", {
            return vdot(-facing:forevector, ship:bounds:furthestcorner(facing:forevector) - ship:position).
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
            local totalMassFlow is 0.
            for eng in engines {
                set totalMassFlow to totalMassFlow + eng:maxmassflow * eng:thrustLimit / 100.
            }
            return totalMassFlow.
        },

        "retrogradeAeroforceat", {
            parameter a, speed.
            print body:atm:altitudepressure(a).
            return addons:far:aeroforceat(a, -facing:forevector * speed):mag.
        }
    ).
}
