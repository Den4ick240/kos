
function profileName {
    parameter dir is "profiles".
    parameter suffix is "".
    local procTag is core:tag.
    if procTag = "" {
        set procTag to "test".
    }
    local jsonPath is "0:/" + dir + "/" + procTag + suffix + ".json".
    return jsonPath.
}
