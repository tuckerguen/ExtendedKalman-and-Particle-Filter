classdef Landmark
    %LANDMARK Represents a landmark on the map
    %   Has x and y location, and ms signature
    
    properties
        mx        % Map location x
        my        % Map location y
        ms        % True Id
    end
    
    methods
        function obj = Landmark(mx, my, ms)
            %LANDMARK Construct an instance of this class
            %   Detailed explanation goes here
            obj.mx = mx;
            obj.my = my;
            obj.ms = ms;
        end
    end
end

