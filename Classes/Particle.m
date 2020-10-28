classdef Particle
    %PARTICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x State % State hypothesis of this particle
        w       % Weight of the particle
    end
    
    methods
        function obj = Particle(x, w)
            %PARTICLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x;
            obj.w = w;
        end
    end
end

