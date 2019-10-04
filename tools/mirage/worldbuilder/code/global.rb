# Tekkotsu WorldBuilder
# Global variables
#
# Robert Lee (rslee / perihare)
# 2-14-2011

require 'parsevector'

# Core data structure which holds all world information
$map = Hash.new

# Arrays holding each type of object in the world
$observer = nil
$background = nil
$shadows = nil
$physics = nil
$lights = []
$shapes = []

# Global binding contexts, for shapes (defines) and numbers (context)
$defines = Hash.new
$definedNames = []
$context = Hash.new

# Put pi in the context, by default.
# (This means you *could* redefine pi, if you really wanted to...)
$context['pi'] = Math::PI

# Counters for how many shapes/lights exist in the world
$shapecount = 0
$lightcount = 0

# Logo variables
$pendown = false
$position = ParseVector.new(0, 0, 0)
$penshape = "Cube"
$angle = 0.0
$penwidth = 10
$penheight = 0.01
$penmaterial = "UnknownMaterial"

# Parser variables
$linecount = 0
