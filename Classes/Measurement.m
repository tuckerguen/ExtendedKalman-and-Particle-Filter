classdef Measurement
    %MEASUREMENT Represents a sensor measurement 
    %   for the landmark sensing model
    
    properties
        r           % distance to landmark
        phi         % bearing to landmark
        c           % landmark ID
    end
    
    methods
        function obj = Measurement(r, phi, c)
            %MEASUREMENT Construct an instance of this class
            %   Detailed explanation goes here
            obj.r = r;
            obj.phi = phi;
            obj.c = c;
        end
        
        function vec = get_measurement_vec(obj)
            vec = double([obj.r; obj.phi; obj.c]);
        end
    end
end

