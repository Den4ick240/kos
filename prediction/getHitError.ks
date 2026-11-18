
function getHitError {
    parameter selfVector, targetVector, landing.
    parameter _body is ship:body.

    local predictedTargetVector to angleaxis(
        _body:angularvel:mag * constant:radtodeg * landing:time, 
        _body:angularvel
    ) * targetVector.

    local downrangeDir is vxcl(predictedTargetVector, selfVector).
    if downrangeDir:sqrmag > 0.00001 {
        set downrangeDir to downrangeDir:normalized.
    } else {
        
    }
    local crossrangeDir is vcrs(predictedTargetVector, downrangeDir):normalized.

    local err is landing:pos - predictedTargetVector.
    local downrangeErr is vdot(err, downrangeDir).
    local crossrangeErr is vdot(err, crossrangeDir).

    return lex(
        "downrange", downrangeErr,
        "crossrange", crossrangeErr,
        "downrangeDir", downrangeDir,
        "crossrangeDir", crossrangeDir
    ).
}
