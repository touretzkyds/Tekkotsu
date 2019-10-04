# Tekkotsu WorldBuilder
# Logo turtle scripting support
#
# Robert Lee (rslee / perihare)
# 2-14-2011

# Globals
require 'global'

# Class definitions
require 'shape'
require 'parsevector'

def parse_logo(logo_command)
    command = logo_command[0]
    logo_value = logo_command[1]
    
    case command
    when "penup"
        $pendown = false
    when "pendown"
        $pendown = true
    when "penshape"
        $penshape = logo_value
    when "pencolor"
        $penmaterial = logo_value
    when "penwidth"
        $penwidth = logo_value
    when "penheight"
        $penheight = logo_value
    when "forward"
        distance = logo_value
        newpos = $position.clone
        newpos.x = newpos.x.to_f + distance * Math.cos($angle)
        newpos.y = newpos.y.to_f + distance * Math.sin($angle)
        makeshape(distance, newpos, false) if $pendown
        $position = newpos
    when "turn"
        $angle = $angle.to_f + (logo_value * Math::PI / 180.0)
    when "up"
        distance = logo_value
        newpos = $position.clone
        newpos.z = newpos.z.to_f + distance
        makeshape(distance, newpos, true) if $pendown
        $position = newpos
    when "moveto"
        $position = logo_value
    when "heading"
        $angle = logo_value * Math::PI / 180.0
    else
        puts "Implementation error:"
        puts "LOGO parsed correctly, but encountered an unknown keyword."
    end
end

def makeshape(dist, newpos, up)
    s = Shape.new
    s.name = "shape-%03d" % ($shapecount = $shapecount + 1)
    center = $position.clone
    center.x = (center.x.to_f + newpos.x.to_f) / 2.0
    center.y = (center.y.to_f + newpos.y.to_f) / 2.0
    center.z = (center.z.to_f + newpos.z.to_f) / 2.0
    s.set_attr("type", $penshape)
    s.set_attr("location", center)
    s.set_attr("material", $penmaterial)
    if up
        s.set_attr("scale", ParseVector.new($penwidth,$penheight,dist))
    else
        s.set_attr("scale", ParseVector.new(dist,$penwidth,$penheight))
        s.set_attr("mrotation", ParseVector.new(0,0,get_angle))
    end
    $shapes.push s
end

def get_angle
    angle = $angle
    while angle < 0
        angle = angle.to_f + Math::PI * 2
    end
    while angle >= Math::PI * 2
        angle = angle.to_f - Math::PI * 2
    end
    q = Math.sin(angle / 2.0)
    q = -q if angle > Math::PI
    return q
end
