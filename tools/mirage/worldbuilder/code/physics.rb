# Tekkotsu WorldBuilder
# Physics class definition
#
# Robert Lee (rslee / perihare)
# 2-14-2011

class Physics
    attr_accessor :erp, :gravity, :mass, :solver, :space, :steps

    def set_attr(attr, val)
        case attr
        when "erp"
            @erp = val
        when "gravity"
            @gravity = val
        when "massscale"
            @mass = val
        when "solveriterations"
            @solver = val
        when "spacescale"
            @space = val
        when "stepsperframe"
            @steps = val
        else
            puts "Invalid attribute #{attr}"
        end
    end

    def to_map
        h = Hash.new
        h["ERP"] = @erp.to_f unless @erp.nil?
        h["Gravity"] = @gravity.to_f unless @gravity.nil?
        h["MassScale"] = @mass.to_f unless @mass.nil?
        h["SolverIterations"] = @solver.to_i unless @solver.nil?
        h["SpaceScale"] = @space.to_f unless @space.nil?
        h["StepsPerFrame"] = @steps.to_i unless @steps.nil?
        h
    end
end
