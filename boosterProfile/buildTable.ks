// steps for is [x0, step0_1, x1, step1_2, x2 ... stepn-1_n, xn].
function build1dTable {
    parameter steps, fun.

    local end is steps[steps:length - 1].
    local table is list().
    local a is steps[0].
    local i is 1.
    until a > end {
        table:add(fun(a)).
        if i + 2 < steps:length and a > steps[i + 1] {
            set i to i + 2.
        }
        set a to a + steps[i].
    }
}

function build2dTable {
    parameter steps1, steps2, fun.

    return build1dTable(
        steps1, {
            parameter a.
            return build1dTable(
                steps2, {
                    parameter b.
                    return fun(a, b).
                }
            )
        }
    )
}
