# Tekkotsu WorldBuilder
# Syntax node classes for the various expression grammars
#
# Robert Lee (rslee / perihare)
# 3-7-2011
#
# Based on the Treetop example code:
# https://github.com/nathansobo/treetop
#
# Dijkstra's shunting-yard algorithm based on work from:
# http://austintaylor.org/blog/2010/07/25/treetop-operator-precedence/
# This work has been extended to support unary operators (negation and trig functions.)

module ExpressionGrammar
    @function_strings = ['sin', 'cos', 'tan', 'sqrt', 'abs', 'deg2rad']
    @operator_strings = ['+', '-', '*', '/', '%', '^', '==', '<', '>']
    # Delta value used for comparing floats for 'equality'
    @delta = 0.001

    Operator = Struct.new(:name, :unary, :pre, :assoc)
    @operators = {}

    def self.op(associativity, unary, *operators)
        @precedence ||= 0
        @operators ||= {}
        operators.each do |operator|
            @operators[operator] = Operator.new(operator, unary, @precedence, associativity)
        end
        @precedence += 1
    end
    
    def self.apply_op(operator, a, b)
        case operator.name
        when '+' then return a + b
        when '-' then return a - b
        when '*' then return a * b
        when '/' then return a / b
	when '%' then return a % b
        when '^' then return a ** b
        when '==' then return ((a - b).abs < @delta) ? 1 : 0
        when '<' then return (a < b) ? 1 : 0
        when '>' then return (a > b) ? 1 : 0
        else puts 'Invalid operator: ' + operator.name
        end
    end
    
    def self.apply_unary_op(operator, a)
        case operator.name
        when '~' then return -1 * a
        when 'sin' then return Math.sin(a)
        when 'cos' then return Math.cos(a)
        when 'tan' then return Math.tan(a)
        when 'sqrt' then return Math.sqrt(a)
        when 'abs' then return a.abs
	when 'deg2rad' then return (a * Math::PI / 180)
        else puts 'Invalid operator: ' + operator.name
        end
    end

    # Operator precedence table, arranged from low to high.
    op :left, false, '||'
    op :left, false, '&&'
    op :none, false, '==', '!='
    op :left, false, '<', '<=', '>', '>='
    op :right, true, 'sin', 'cos', 'tan', 'sqrt', 'abs', 'deg2rad'
    op :left, false, '+', '-'
    op :left, false, '*', '/', '%'
    op :right, false, '^'
    op :right, true, '~'
    
    def self.shunting_yard(yard)
        output = []
        stack = []
        # Store the previous expression to handle unary minus.
        previous_e = nil
        yard.each do |e|
            # Push numbers directly onto the output queue.
            if e.kind_of? Float then
                output.push e
            # Push functions directly onto the stack.
            elsif @function_strings.include? e then
                stack.push e
            # Push and pop operators on the stack based on order of precedence.
            elsif @operator_strings.include? e then
                e1 = @operators[e]
                e2 = stack.last
                # Special case: We find a unary minus when a minus sign has
                # another operator, a left paren, or nothing to its left.
                if (e.eql? '-') && (previous_e.nil? ||
                    (previous_e.eql? '(') ||
                    (@operator_strings.include? previous_e)) then
                    e1 = @operators['~']
                end
                while @operators.has_value? e2 do
                    if ((e1[:assoc] == :left && e1[:pre] <= e2[:pre]) ||
                        (e1[:assoc] == :right && e1[:pre] < e2[:pre])) then
                        output.push stack.pop
                        e2 = stack.last
                    else
                        break
                    end
                end
                stack.push e1
            # Push left parens directly onto the stack.
            elsif e.eql? '(' then
                stack.push :leftparen
            # Right parens pop operators off the stack until reaching
            # their matching left paren.
            elsif e.eql? ')' then
                until stack.empty? || stack.last == :leftparen do
                    output.push stack.pop
                end
                unless stack.empty? then
                    stack.pop
                    if @function_strings.include? stack.last then
                        output.push @operators[stack.pop]
                    end
                else
                    puts 'Mismatched right parentheses in expression: ' + yard.to_s
                    return nil
                end
            end   
            previous_e = e     
        end
        
        # Pop all remaining operators off the stack.
        until stack.empty?
            op = stack.pop
            if op == :leftparen || op == :rightparen then
                puts 'Mismatched parentheses in expression: ' + yard.to_s
                return nil
            end
            output.push op
        end
        
        return rpn(output, yard)
    end
    
    def self.rpn(expr, yard)
        result = []
        expr.each do |e|
            if @operators.has_value? e then
                if e.unary then
                    a = result.pop
                    if a.nil? then
                        puts 'Unmatched negation in expression: ' + yard.to_s
                        return nil
                    end
                    result.push(apply_unary_op(e, a))
                else
                    b = result.pop
                    a = result.pop
                    if a.nil? || b.nil? then
                        puts 'Not enough arguments in expression: ' + yard.to_s
                        return nil
                    end
                    result.push(apply_op(e, a, b))
                end
            else
                result.push e
            end
        end
        if result.length > 1 then
            puts 'Too many values in expression: ' + expr.to_s
            return nil
        end
        return result.first
    end
    
end

module ShapeGrammar
    class ShapeParameter < Treetop::Runtime::SyntaxNode
        def eval(env={})
            return {keyword.text_value => value.eval(env)}
        end
    end
end

module LightGrammar
    class LightParameter < Treetop::Runtime::SyntaxNode
        def eval(env={})
            return {keyword.text_value => value.eval(env)}
        end
    end
end

module EnvironmentGrammar
    class EnvironmentCommand < Treetop::Runtime::SyntaxNode
        def eval(env={})
            m = Hash.new
            parameters.elements.each do |p|
                m = m.merge p.eval(env)
            end
            return [keyword.text_value, m]
        end
    end
    
    class EnvironmentParameter < Treetop::Runtime::SyntaxNode
        def eval(env={})
            return {keyword.text_value => value.eval(env)}
        end
    end
end
