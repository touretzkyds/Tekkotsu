# Tekkotsu WorldBuilder
# Shape class definition
#
# Robert Lee (rslee / perihare)
# 2-14-2011

require 'parsevector'
require 'global'

require 'boolean'

class Shape
    attr_accessor :name, :parent, :attachto, :attachments, :components, :cached_map, :location, :group, :attrs, :lineno 
    
    def initialize
        @parent = nil
        @attachto = nil
        @attachments = []
        @components = []
        @lineno = $linecount
        @attrs = {}
        @valid_attrs ||= ['name', 'orientation', 'pointat',
            'type', 'kinematics', 'scale', 'material', 'mass',
            'model', 'moffset', 'mrotation', 'mscale',
            'cmodel', 'cmoffset', 'cmrotation', 'cmscale',
            'centerofmass', 'visible', 'collision', 'group']
        @attrs['collision'] = 'true' # shapes collide by default
        @group = []
        @group.push "all"
        @group.push "real"
    end

    def clone
        s = Shape.new
        s.name = @name
        s.parent = @parent
        s.attachto = @attachto
        s.components = @components.clone
        @group.each do |groupname|
            s.set_attr('group', groupname)
        end
        @attrs.each do |attr, val|
            s.set_attr(attr, val)
        end
        s.location = @location.clone unless @location.nil?
        return s 
    end

    def set_attr(attr, val)
        if attr.eql? 'attachto' then
            if @attachto.nil?
                @attachto = val
            else
                puts "Shape %s defined at line %d has multiple attachto values." % [@name, @lineno]
                abort "Can't properly define " + @name
            end
        elsif attr.eql? 'group' then
            @group.push val.clone
        elsif attr.eql? 'location' then
            @location = val
        elsif attr.eql? 'mass' then
            @attrs[attr] = val
        elsif @valid_attrs.include? attr then
            @attrs[attr] = val.clone
        else
            raise "Invalid argument: " + attr.to_s + ", " + val.to_s
        end
    end

    def to_map
        return @cached_map if ! @cached_map.nil?
        h = Hash.new
        @cached_map = h  # do this early, in case of loop in attachto graph
        h["Location"] = @location.to_array unless @location.nil?
        h["Orientation"] = attrs['orientation'].to_array unless attrs['orientation'].nil?
        h["PointAt"] = attrs['pointat'].to_array unless attrs['pointat'].nil?
        h["Kinematics"] = attrs['kinematics'] unless attrs['kinematics'].nil?
        h["CenterOfMass"] = attrs['centerofmass'].to_array unless attrs['centerofmass'].nil?
        if to_boolean(attrs['collision']) then
            h["CollisionModel"] = (attrs['cmodel'].nil?) ? attrs['type'].capitalize : attrs['cmodel']
            # use model parameters as defaults
            h["CollisionModelOffset"] = attrs['moffset'].to_array unless attrs['moffset'].nil?
            h["CollisionModelRotation"] = attrs['mrotation'].to_array unless attrs['mrotation'].nil?
            h["CollisionModelScale"] = attrs['scale'].to_array unless attrs['scale'].nil?
            # override with cmodel parameters if supplied
            h["CollisionModelOffset"] = attrs['cmoffset'].to_array unless attrs['cmoffset'].nil?
            h["CollisionModelRotation"] = attrs['cmrotation'].to_array unless attrs['cmrotation'].nil?
            h["CollisionModelScale"] = attrs['cmscale'].to_array unless attrs['cmscale'].nil?
        end
        h["Model"] = (attrs['model'].nil?) ? attrs['type'].capitalize : attrs['model']
        h["ModelOffset"] = attrs['moffset'].to_array unless attrs['moffset'].nil?
        h["ModelRotation"] = attrs['mrotation'].to_array unless attrs['mrotation'].nil?
        h["ModelScale"] = attrs['scale'].to_array unless attrs['scale'].nil?
        h["ModelScale"] = attrs['mscale'].to_array unless attrs['mscale'].nil?
        h["Mass"] = attrs['mass'] unless attrs['mass'].nil?
        h["Material"] = attrs['material'] unless attrs['material'].nil?
        h["Visible"] = to_boolean(attrs['visible']) unless attrs['visible'].nil?

        if ! @parent.nil?
            $defines[@parent].to_map
            @components = $defines[@parent].components.clone
        end
        @attachments.each {|a| @components.push(a.to_map) }
        h["Components"] = @components unless @components.empty?
        # puts "to_Map: " + @name + "  parent=" + (@parent||"nil") + "  attachto=" +
        #   (@attachto.nil? ? "nil" : @attachto) + "  attachments=" +  (@attachments.nil? ? "[]" : @attachments).to_s
        return h
    end

    def doattach
        if ! @attachto.nil?
            attached = $defines[@attachto]
            if attached.nil?
                raise "Can't attach shape %s to non-existent shape %s" % [@name, @attachto]
            else
                attached.attachments.push(self)
                # puts "Attaching " + @name + " to " + @attachto + " yielding " + attached.attachments.to_s
            end
        end
    end

    def genname
        $shapecount = $shapecount + 1
        @name = "shape-%03d" % $shapecount
        extraname = @attrs['name'] || @attrs['type']
        @name = @name + " " + extraname unless extraname.nil?
    end

    def flipx
        location.y = -location.y.to_f
        newcmrotation = attrs['cmrotation']
        if ! newcmrotation.nil?
          newcmrotation.y = -newcmrotation.y.to_f
          set_attr('cmrotation', newcmrotation)
        end
    end
    
    def flipy
        location.x = -location.x.to_f
        newcmrotation = attrs['cmrotation']
        if ! newcmrotation.nil?
          newcmrotation.x = -newcmrotation.x.to_f
          set_attr('cmrotation', newcmrotation)
        end
    end
end
