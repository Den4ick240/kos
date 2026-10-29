function advanceRk4 {
    parameter pos, vel, acc, getAcc, dt.

    local halfDt is dt / 2.
    local dtSqrdOver8 is dt * dt / 8.
    local dtOver6 is dt / 6.
    
    local pos2 is pos + vel * halfDt + acc * dtSqrdOver8.
    local vel2 is vel + acc * halfDt.
    local acc2 is getAcc(pos2, vel2).

    local pos3 is pos2.
    local vel3 is vel + acc2 * halfDt.
    local acc3 is getAcc(pos3, vel3).

    local pos4 is pos + vel * dt + acc3 * dt * halfDt.
    local vel4 is vel + acc3 * dt.
    local acc4 is getAcc(pos4, vel4).

    return lex(
        "pos", pos + vel * dt + (acc + acc2 + acc3) * dt * dtOver6,
        "vel", vel + (acc + acc2 * 2 + acc3 * 2 + acc4) * dtOver6
    ).
}

function advanceRk4WithMass {
    parameter pos, vel, m, acc, getAcc, massFlow, dt.

    local halfDt is dt / 2.
    local dtSqrdOver8 is dt * dt / 8.
    local dtOver6 is dt / 6.

    local mass2 is max(0.1, m - massFlow * halfDt).
    local pos2 is pos + vel * halfDt + acc * dtSqrdOver8.
    local vel2 is vel + acc * halfDt.
    local acc2 is getAcc(pos2, vel2, mass2).

    local mass3 is max(0.1, m - massFlow * halfDt).
    local pos3 is pos2.
    local vel3 is vel + acc2 * halfDt.
    local acc3 is getAcc(pos3, vel3, mass3).

    local mass4 is max(0.1, m - massFlow * dt).
    local pos4 is pos + vel * dt + acc3 * dt * halfDt.
    local vel4 is vel + acc3 * dt.
    local acc4 is getAcc(pos4, vel4, mass4).

    return lex(
        "pos", pos + vel * dt + (acc + acc2 + acc3) * dt * dtOver6,
        "vel", vel + (acc + acc2 * 2 + acc3 * 2 + acc4) * dtOver6,
        "m", max(0.1, m - massFlow * dt)
    ).
}
