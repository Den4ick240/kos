
function boosterProfileName {
    local procTag is core:tag.
    if procTag = "" {
        set procTag to "test".
    }
    local jsonPath is "0:/bosterprofiles/" + procTag + ".json".
    return jsonPath.
}
