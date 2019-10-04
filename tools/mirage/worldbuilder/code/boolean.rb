# Tekkotsu WorldBuilder
# String-to-boolean conversion utility
#
# Robert Lee (rslee / perihare)
# 2-14-2011

def to_boolean(s)
    return ["true", "t", "yes", "1", "okay", "sure"].include? s.downcase
end