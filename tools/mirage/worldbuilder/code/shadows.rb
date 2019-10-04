# Tekkotsu WorldBuilder
# Shadows class definition
#
# Robert Lee (rslee / perihare)
# 2-14-2011

require "color"
require "boolean"

class Shadows
    attr_accessor :color, :enabled, :mod, :stencil

    def set_attr(attr, val)
        case attr
        when "color"
            @color = val
        when "enabled"
            @enabled = val
        when "modulative"
            @mod = val
        when "stencil"
            @stencil = val
        else
            puts "Invalid attribute #{attr}"
        end
    end

    def to_map
        h = Hash.new
        h["Color"] = @color.to_s unless @color.nil?
        h["Enabled"] = to_boolean(@enabled) unless @enabled.nil?
        h["Modulative"] = to_boolean(@mod) unless @mod.nil?
        h["Stencil"] = to_boolean(@stencil) unless @stencil.nil?
        h
    end
end
