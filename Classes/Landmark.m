classdef Landmark
    %LANDMARK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mx        % Map location x
        my        % Map location y
    end
    
    methods
        function obj = Landmark(mx, my)
            %LANDMARK Construct an instance of this class
            %   Detailed explanation goes here
            obj.mx = mx;
            obj.my = my;
        end
    end
end

