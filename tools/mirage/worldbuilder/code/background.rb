# Tekkotsu WorldBuilder
# Background class definition
#
# Robert Lee (rslee / perihare)
# 2-14-2011

class Background
    attr_accessor :color, :curv, :dist, :material, :model, :tiling 

    def set_attr(attr, val)
        case attr
        when "color"
            @color = val
        when "curvature"
            @curv = val
        when "distance"
            @dist = val
        when "material"
            @material = val
        when "model"
            @model = val
        when "tiling"
            @tiling = val
        else
            puts "Invalid attribute #{attr}"
        end
    end

    def to_map
        h = Hash.new
        h["Color"] = @color.to_s unless @color.nil?
        h["Curvature"] = @curv.to_f unless @curv.nil?
        h["Distance"] = @dist.to_f unless @dist.nil?
        h["Material"] = @material unless @material.nil?
        h["Model"] = @model.capitalize unless @model.nil?
        h["Tiling"] = @tiling.to_i unless @tiling.nil?
        h
    end
end
