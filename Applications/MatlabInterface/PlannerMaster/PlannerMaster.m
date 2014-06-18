% PLANNER MASTER
%   MATLAB class wrapper to the Node C++ architecture.
%   This MATLAB program works simultaneously with the program
%   PathPlanner (./Applications/PathPlanner) to solve Boundary
%   Value Problems (BVP). 

classdef PlannerMaster < handle
    
    properties (SetAccess = public)
        planners_;         % Handle to the node instance
        num_planners_;
        bvp_solutions_;
        bvp_splines_;
        available_sims_;
        busy_sims_;
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% CONSTRUCTOR/DESTRUCTOR
        
        function this = PlannerMaster(num_planners)
        % This gives us the global methods we need. 
            addpath('../');
            this.planners_ = PlannerMaster_mex('new', num_planners);
            % TODO: Init all possible start/goal configurations
            this.num_planners_ = num_planners;
            available_sims_ = [0:1:num_planners_];
            busy_sims_ = [];
        end
        
        function delete(this)
            PlannerMaster_mex('delete', this.planners_);
        end
        
        function CreateLookupTable
            tau = 0;             
            granularity = 10; % creates a 31 x 21 matrix
            scale = 3;
            smoothness = 3;
            is_recorded = false;
            mesh = GenMesh(tau, granularity, scale, smoothness, is_recorded);
            % make an array of start and goal points
            start_theta = [-.4 : .2 : .4]; % 5 possible thetas
            start_vel = [.5: .5 : 2.5]; % 5 possible start velocities
            [starts] = combvec(start_theta, start_vel);
            start_x_y = repmat([0;0], 1, numel(starts(1, :))); 
            start_point = [start_x_y; pairs];
            goal_x = [3 : 1 : 6]; % 4 x values
            goal_y = [-2 : 1 : 2]; % 5 y values
            goal_theta = [-.4 : .2 : .4]; % 5 theta values
            goal_vel = [1 : 1 : 3]; % 3 vel values
            
            %% Starting with something simple
            start_theta = [-.4 : .2 : -.4]; % 5 possible thetas
            start_vel = [.5: .5 : .5]; % 5 possible start velocities
            [starts] = combvec(start_theta, start_vel);
            start_x_y = repmat([0;0], 1, numel(starts(1, :))); 
            start_point = [start_x_y; pairs];
            goal_x = [3 : 1 : 3]; % 4 x values
            goal_y = [-2 : 1 : 2]; % 5 y values
            goal_theta = [-.4 : .2 : -.4]; % 5 theta values
            goal_vel = [1 : 1 : 3]; % 3 vel values
            
            
            [goals] = combvec(goal_x, goal_y, goal_theta, goal_vel);
            % We have a lot of policies to go through; about 7500, 
            % in fact. We can do it!
            for ii = 0:numel(start_point(:,1)), 
                for jj = 0:numel(start_point(:,1)), 
                    avail_sim = this.GetAvailableSim();
                    while avail_sim == -1, 
                        avail_sim = this.GetAvailableSim();
                    end
                    is_found = this.FindSinglePolicy(start_point(:, ii), ...
                                                     goal_point(:, jj), ...                                                     
                                                     mesh, ...
                                                     avail_sim);
                end
            end
            % We should have a huge matrix of all of our solutions now
            filename = ['PathSolutions'];
            save(filename, this.bvp_splines_, ' ');
        end 
        
        function [is_found] = FindSinglePolicy(start_point, goal_point, ...
                                               mesh, sim_num)
            % This function only uses the PathPlanner called Sim0, 
            % and only tests one path on one mesh. 
            % Parameters for GenMesh
            this.SetBusy(sim_num);
            is_found = false;
            while 1, 
                [config_stat, mesh_stat, policy_stat] = this.GetStatus(sim_num); 
                if config_stat == 0 && mesh_stat == 0, 
                    disp('Sending BVP Now');
                    success = this.SetBVP(sim_num, start_point, goal_point, ...
                                          mesh);
                elseif policy_stat == 1, 
                    disp('Getting Policy'); 
                    [policy] = this.GetPolicy(sim_num);
                    [spline] = this.GetSpline(sim_num);
                    this.SaveSpline(tau, start_point, goal_point, ...
                                    mesh, spline);
                    is_found = true; 
                    this.SetAvailable(sim_num);
                    break;
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% MATLAB FUNCTIONS
        %%%% These don't interact with node at all. 
        
        %%% Save our Policy to a .mat file
        function SavePolicy(this, tau, start_point, goal_point, mesh, policy)
            BVPSolution.force = policy.force;
            BVPSolution.phi = policy.phi;
            BVPSolution.duration = policy.duration;
            BVPSolution.meshX = mesh.X;
            BVPSolution.meshY = mesh.Y;
            BVPSolution. meshZ = mesh.Z;
            BVPSolution.start_config = start_point;
            BVPSolution.goal_config = goal_point;
            BVPSolution.map_bit = tau;
            this.bvp_solutions_ = [this.bvp_solutions_, ...
                                BVPSolution];
            % Saving is a pain in the ass
            bvpsol = this.bvp_solutions_;
            filename = ['BVPSolution-' num2str(tau) '.mat'];
            save(filename, 'bvpsol');
        end
        
        %%% Save our spline function to a .mat file
        function SaveSpline(this, tau, start_point, goal_point, mesh, spline)
            BVPSolution.spline = spline;
            BVPSolution.meshX = mesh.X;
            BVPSolution.meshY = mesh.Y;
            BVPSolution. meshZ = mesh.Z;
            BVPSolution.start_config = start_point;
            BVPSolution.goal_config = goal_point;
            BVPSolution.map_bit = tau;
            this.bvp_splines_ = [this.bvp_splines_, ...
                                BVPSolution];
            % Saving is a pain in the ass
            bvp_splines = this.bvp_splines_;
            filename = ['BVPSplines-' num2str(tau) '.mat'];
            save(filename, 'bvp_splines');
        end
        
        
        %%% Save our spline function to a .txt file
        function SaveSpline(this, tau, start_point, goal_point, mesh, spline)
            BVPSolution.spline = spline;
            % Each solution takes up one row in this
            % space-delimited file. The order of variables: 
            % mesh.zz (31x21), start (theta, vel), goal (x, y, th,
            % vel), spline (6 x,y coordinates in order. This should
            % make a row of 669 elements           
            BVPSolution = [BVPSolution, mesh.zz_row];
            BVPSolution = [BVPSolution, start_point(3)];
            BVPSolution = [BVPSolution, start_point(4)];
            BVPSolution = [BVPSolution, goal_point(1)];
            BVPSolution = [BVPSolution, goal_point(2)];
            BVPSolution = [BVPSolution, goal_point(3)];
            BVPSolution = [BVPSolution, goal_point(4)];
            for ii=1:numel(spline.x_values), 
                BVPSolution = [BVPSolution, spline.x_values(ii)];
                BVPSolution = [BVPSolution, spline.y_values(ii)];
            end        
            this.bvp_splines_ = [this.bvp_splines; BVPSolution];
        end
        
        %% SetBusy and SetAvailable create a ring buffer of available
        %% and busy functions, helping us to move simulation along
        function SetBusy(this, sim_num)
            avail = find(this.available_sims_ == sim_num);
            busy = find(this.busy_sims_ == sim_num);
            if isempty(avail), 
                if isempty(busy), 
                    this.busy_sims_ = [this.busy_sims_, sim_num];
                end
            else
                this.available_sims_(avail) = []; % removes them
                                                  % from array
            end
        end
        
        function SetAvailable(this, sim_num)
            avail = find(this.available_sims_ == sim_num);
            busy = find(this.busy_sims_ == sim_num);
            if isempty(busy), 
                if isempty(avail), 
                    this.available_sims_ = [this.avaliable_sims_, sim_num];
                end
            else
                this.busy_sims_(busy) = []; % removes them
                                            % from array
            end
        end
        
        function [sim_num] = GetAvailableSim(this)
            if isempty(this.available_sims_), 
                return -1; 
            else
                return this.available_sims_(0);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% NODE FUNCTIONS
        
        %%% ...cough
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% SETTERS AND GETTERS
        %%%% All of these send with the node name "MATLAB"
        
        function [success] =  SetConfiguration(this, planner_num, start_state, ...
                                               goal_state)
            success = PlannerMaster_mex('SetConfiguration', ...
                                        this.planners_, planner_num, ...
                                        start_state, goal_state);
        end
        
        function [success] = SetHeightmap(this, planner_num, mesh)
            success = PlannerMaster_mex('SetHeightmap', this.planners_, ...
                                        planner_num, mesh.xx, mesh.yy, mesh.zz, ...
                                        mesh.row_count, mesh.col_count);
        end    

        function [success] = SetBVP(this, planner_num, start_state, goal_state, ...
                                    mesh)
            disp('Setting Configuration');
            config_success = this.SetConfiguration(planner_num, start_state, ...
                                                   goal_state);
            pause(2);
            disp('Setting Mesh');
            mesh_success = this.SetHeightmap(planner_num, mesh);
            pause(2);
            success = 0;
            if config_success == 1 && mesh_success == 1, 
                success = 1;
            end
        end
        
        function [config_status, mesh_status, policy_status] = GetStatus(this, planner_num)
            [config_status, mesh_status, policy_status] = PlannerMaster_mex('GetStatus', this.planners_, planner_num);        
        end
        
        function [policy] = GetPolicy(this, planner_num)
            [force, phi, time] = PlannerMaster_mex('GetPolicy', this.planners_, planner_num);
            policy.force = force;
            policy.phi = phi;
            policy.duration = time;
        end
        
        function [motion_sample] = GetMotionSample(this, planner_num)
            [x, y, z, r, p, q] = PlannerMaster_mex('GetMotionSample', this.planners_, ...
                                                   planner_num);
            motion_sample.x = x;
            motion_sample.y = y;
            motion_sample.z = z;
            motion_sample.r = r;
            motion_sample.p = p;        
            motion_sample.q = q;
        end        
        
        function [spline] = GetSpline(this, planner_num)
            [x_values, y_values, solved_goal_pose] = ...
                PlannerMaster_mex('GetSpline', this.planners_, ...
                              planner_num);
            spline.x_values = x_values;
            spline.y_values = y_values;
            spline.solved_goal_pose = solved_goal_pose;
        end
        
        
    end
end
