classdef StateBelief
    %StateBelief Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mu State        % x, y, theta mean of belief
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
        
        function [locs, covs] = to_matrices(beliefs)
            n = length(beliefs);
            covs = zeros(3, 3, n);
            locs = zeros(n, 3);
            for i = 1:n
                bel = beliefs(i);
                loc = get_loc(bel);
                locs(i,:) = loc';
                cov = bel.Cov;
                covs(:,:,i) = cov;
            end
        end
        
        function [plots, legend] = plot_beliefs(beliefs, is_pre_update, color)
            n = length(beliefs);
            [locs, covs] = to_matrices(beliefs);
            if ~is_pre_update
                locxs = locs(:,1);
                locys = locs(:,2);
                loc_plot = plot(locxs, locys, 'm');
            end

            for i=1:n
                mu_i = get_vec(beliefs(i).mu);
                mu_i = mu_i(1:2);
                cov = covs(2:3,2:3,i);
                % https://www.mathworks.com/matlabcentral/fileexchange/16543-plot_gaussian_ellipsoid
                ellipse = plot_gaussian_ellipsoid(mu_i, cov, 1);
                set(ellipse,'color',color); 
            end
            
            if is_pre_update
                plots = [ellipse];
                legend = ["pre-update covariance"];
            else
                plots = [loc_plot; ellipse];
                legend = ["estimated pose"; "post-update covariance"];
            end
        end
    end
end

