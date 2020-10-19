classdef Command
    %COMMAND Represents a motion command
    %   Motion command of the form: [v, w]
    
    properties
        v           % translational velocity
        w           % rotational velocity
    end
    
    methods
        function obj = Command(v, w)
            %COMMAND Construct an instance of this class
            obj.v = v;
            obj.w = w;
        end
    end
end

