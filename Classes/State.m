classdef State
    %STATE Represents a planar robot state
    
    properties
        x   % x position
        y   % y position
        t   % heading measured from +x axis counterclockwise
    end
    
    methods
        function obj = State(x, y, t)
            %STATE Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x;
            obj.y = y;
            obj.t = t;
        end
        
        function xtp1 = add(obj, s2)
            xtp1 = State(obj.x + s2.x, obj.y + s2.y, obj.t + s2.t);
        end

       function sum = add_vector(obj, vals)
            sum = State(obj.x + vals(1), obj.y + vals(2), obj.t + vals(3));
        end
        
        function vec = get_vec(obj)
            vec = [obj.x; obj.y; obj.t];
        end
    end
end

