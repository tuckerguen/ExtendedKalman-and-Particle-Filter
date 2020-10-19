classdef Map
    %MAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        landmarks containers.Map % List of landmarks
    end
    
    methods
        function obj = Map(IDs, landmarks)
            %MAP Construct an instance of this class
            %   Detailed explanation goes here
            obj.landmarks = containers.Map(IDs, landmarks);
        end
    end
end

