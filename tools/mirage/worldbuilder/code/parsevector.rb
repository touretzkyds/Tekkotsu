# Tekkotsu WorldBuilder
# Generalized container for 3 floats
#
# Robert Lee (rslee / perihare)
# 2-14-2011

class ParseVector
    attr_accessor :x, :y, :z

    def initialize(nx = 0, ny = 0, nz = 0)
        @x = nx
        @y = ny
        @z = nz
    end

    def clone
        ParseVector.new(@x, @y, @z)
    end

    def to_array
        [@x.to_f, @y.to_f, @z.to_f]        
    end

    def offset(point)
        @x = @x.to_f + point.x.to_f
        @y = @y.to_f + point.y.to_f
        @z = @z.to_f + point.z.to_f
    end

    def to_s
        return "#{@x},#{@y},#{@z}"
    end
end
