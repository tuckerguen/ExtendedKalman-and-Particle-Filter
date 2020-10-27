classdef StateBelief
    %StateBelief Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mu StateMean        % x, y, theta mean of belief
        Cov(3,3)    % Covariance of belief
    end
    
    methods
        function obj = StateBelief(mu, Cov)
            %StateBelief Construct an instance of this class
            %   Detailed explanation goes here
            obj.mu = mu;
            obj.Cov = Cov;
        end
        
        function loc = get_loc(obj)
            loc = [obj.mu.x; obj.mu.y; obj.mu.t];
        end
    end
end

