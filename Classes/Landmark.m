classdef Landmark
    %LANDMARK Summary of this class goes here
    %   Detailed explanation goes here
    
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

