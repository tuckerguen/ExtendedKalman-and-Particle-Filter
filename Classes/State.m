classdef State
    %STATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        y
        theta
    end
    
    methods
        function obj = State(x, y, theta)
            %STATE Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
        end
        
        function xtp1 = add(obj, s2)
            xtp1 = State(obj.x + s2.x, obj.y + s2.y, obj.theta + s2.theta);
        end
        
        function vec = get_vec(obj)
            vec = [obj.x; obj.y; obj.theta];
        end
    end
end

