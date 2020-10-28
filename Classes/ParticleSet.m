classdef ParticleSet
    %PARTICLESET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        all  % Vector of Particle objects
    end
    
    methods
        function obj = ParticleSet(M)
            %PARTICLESET Construct an instance of this class
            %   Detailed explanation goes here
            obj.all{M} = Particle(State(0,0,0),0);
        end
        
        function obj = insert(obj, particle, i)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.all{i} = particle;
        end
    end
end

