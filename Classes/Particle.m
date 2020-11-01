classdef Particle
    %PARTICLE A state hypothesis with a weight for particle filter
    
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

