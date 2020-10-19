classdef StateMean
    %STATEMEAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x        % mean x of belief
        y        % mean y of belief
        t        % mean theta of belief
    end
    
    methods
        function obj = StateMean(x, y, t)
            %STATEMEAN Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x;
            obj.y = y;
            obj.t = t;
        end
        
        function sum = add_vector(obj, vals)
            sum = StateMean(obj.x + vals(1), obj.y + vals(2), obj.t + vals(3));
        end
    end
end

