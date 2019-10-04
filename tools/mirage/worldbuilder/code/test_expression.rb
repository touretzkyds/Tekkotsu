require 'rubygems'
require 'treetop'
Treetop.load 'expression_grammar'
$e = ExpressionGrammarParser.new
$context = {}
$context['x'] = 1.0
$context['y'] = 2.0
$context['z'] = 3.0
$context['pi'] = Math::PI
$delta = 0.001

def test_expr(expr, expect)
    if $e.parse(expr).nil?
        puts 'Parser failed to parse ' + expr.to_s
        return
    end
    
    value = $e.parse(expr).eval({})
    if (value - expect).abs < $delta then
        puts 'Parser evaluated correctly.'
    else
        puts 'Parser evaluated ' + expr.to_s + ' to ' + value.to_s + ', expected ' + expect.to_s
    end
end

def fail_expr(expr)
    rpn = $e.parse(expr)
    if rpn.nil? || rpn.eval({}).nil? then
        puts 'Parser evaluated correctly.'
    else
        puts 'Parser should have failed: ' + expr.to_s
    end
end

# Single-term tests
test_expr('1', 1)
test_expr('-1', -1)
test_expr('x', 1)
test_expr('-x', -1)

# Variable tests
test_expr('x + y - z', 0)
test_expr('y / z * x', 2.0/3.0)
test_expr('z ^ y ^ x', 9)

# Binary expression tests
test_expr('2+2', 2+2)
test_expr('2+-2', 2+-2)
test_expr('7-4', 7-4)
test_expr('-6*6', -6*6)
test_expr('-144/-12', -144/-12)
test_expr('2^3', 2**3)
test_expr('100 % 33', 100%33)

# Failure cases
fail_expr('')
fail_expr('(')
fail_expr(')')
fail_expr('()')
fail_expr('(()')
fail_expr('())')
fail_expr('+')
fail_expr('-')

# Function applications
test_expr('sin(2*pi)', Math.sin(2*Math::PI))
test_expr('cos(0)', Math.cos(0))
test_expr('abs(-1+2-3+4-5)', (-1+2-3+4-5).abs)
test_expr('sqrt(16)', Math.sqrt(16))
test_expr('tan(pi/4)', Math.tan(Math::PI/4))

# Comparisons
test_expr('5 > 3', 1)
test_expr('5 < 3', 0)
test_expr('1 == 1', 1)
test_expr('1 == 9', 0)
test_expr('(5 > 3) == (3 > 1)', 1)

# Complex expression tests
test_expr('3 + 4 * 2 / ( 1 - 5 ) ^ 2 ^ 3', 3 + 4 * 2 / ( 1.0 - 5 ) ** 2 ** 3)	
test_expr('-3 + -4 * 2 / ( 1 - -5 ) ^ 2 ^ -3', -3 + -4 * 2 / ( 1.0 - -5 ) ** 2 ** -3)	
test_expr('5 + ((1 + 2) * 4 ) - 3', 5 + ((1 + 2) * 4 ) - 3)
test_expr('52 + (1 - 2) * 4 - 3', 52 + (1 - 2) * 4 - 3)
test_expr('sin(((52 + (1 - 2) * 4 - 3) / 360) * 2 * pi)', Math.sqrt(2)/2)
