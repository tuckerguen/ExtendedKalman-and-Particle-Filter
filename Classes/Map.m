classdef Map
    %MAP A map of the object/landmarks in the state space
    %   Stores a map of Landmarks with corresponding ID's
    
    properties
        landmarks containers.Map % Map of ID's to landmarks
    end
    
    methods
        function obj = Map(IDs, landmarks)
            %MAP Construct an instance of this class
            %   Detailed explanation goes here
            obj.landmarks = containers.Map(IDs, landmarks);
        end
    end
end

