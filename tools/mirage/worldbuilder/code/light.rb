# Tekkotsu WorldBuilder
# Light class definition
#
# Robert Lee (rslee / perihare)
# 2-14-2011

require 'parsevector'
require 'global'

class Light
    attr_accessor :name, :location
    
    def initialize
        @name = "light-%02d" % $lightcount
        $lightcount = $lightcount + 1
        @attrs ||= {}
        @valid_attrs ||= ['orientation', 'pointat',
            'color', 'ac', 'al', 'aq', 'ar']
    end

    def clone
        l = Light.new
        @attrs.each do |attr, val|
            s.set_attr(attr, val)
        end
        l.location = @location.clone unless @location.nil?
        return l
    end

    def set_attr(attr, val)
        if attr.eql? 'location' then
            @location = val.clone
        elsif @valid_attrs.include? attr then
            @attrs[attr] = val.clone
        else
            raise "Invalid argument: " + attr.to_s + ", " + val.to_s
        end
    end

    def to_map
        h = Hash.new
        if @location.nil? then
            raise "Light source is missing a location"
        end
        h["Location"] = @location.to_array
        h["Orientation"] = @attrs['orientation'].to_array unless @attrs['orientation'].nil?
        h["PointAt"] = @attrs['pointat'].to_array unless @attrs['pointat'].nil?
        h["AttenuateConst"] = @attrs['ac'] unless @attrs['ac'].nil?
        h["AttenuateLinear"] = @attrs['al'] unless @attrs['al'].nil?
        h["AttenuateQuad"] = @attrs['aq'] unless @attrs['aq'].nil?
        h["AttenuateRange"] = @attrs['ar'] unless @attrs['ar'].nil?
        h["Color"] = @attrs['color'].to_s unless @attrs['color'].nil?
        return h
    end
end
