# Tekkotsu WorldBuilder
# Color class definition
#
# Robert Lee (rslee / perihare)
# 2-14-2011

class Color
    attr_accessor :r, :g, :b

    def initialize(nr = 0, ng = 0, nb = 0)
        @r = nr
        @g = ng
        @b = nb
    end

    def to_s
        # Convert internal integer representation to hex strings
        red = @r.to_s(16)
        green = @g.to_s(16)
        blue = @b.to_s(16)
        if red.length == 1 then red = '0' + red end
        if green.length == 1 then green = '0' + green end
        if blue.length == 1 then blue = '0' + blue end
        "#" + red + green + blue
    end
end
