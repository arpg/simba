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
            this.available_sims_ = [0:1:num_planners-1]
            this.busy_sims_ = [];
        end
        
        function delete(this)
            PlannerMaster_mex('delete', this.planners_);
        end
        
        function CreateLookupTable(this, ii_start)
            tau = 0;             
            granularity = 10; % creates a 31 x 21 matrix
            scale = 3;
            smoothness = 3;
            is_recorded = false;
            mesh = GenMesh(tau, granularity, scale, smoothness, is_recorded);
            % make an array of start and goal points
            start_theta = [-.5 : .1 : .5]; % 5 possible thetas
            start_vel = [.8: .2 : 1.6]; % 5 possible start velocities
            [starts] = combvec(start_theta, start_vel);
            start_x_y = repmat([0;0], 1, numel(starts(1, :))); 
            start_point = [start_x_y; starts];
            goal_x = [4 : .5 : 5.5]; % 4 x values
            goal_y = [-2 : 1 : 2]; % 5 y values
            goal_theta = [-.5 : .2 : .5]; % 5 theta values
            goal_vel = [.8 : .4 : 1.6]; % 3 vel values
            goal_point = combvec(goal_x, goal_y, goal_theta, goal_vel);
            % We have a lot of policies to go through; about 7500, 
            % in fact. We can do it!
            ii = ii_start;
            jj = 1;
            config_stat = zeros(1, this.num_planners_);
            mesh_stat = zeros(1, this.num_planners_);
            policy_stat = zeros(1, this.num_planners_);
            policy_failed = zeros(1, this.num_planners_);
            % These keep track of the points that are in our
            % planners now
            working_start_points = zeros(4, this.num_planners_);
            working_goal_points = zeros(4, this.num_planners_);
            done_planning = false;
            ready_to_save = false;
            saving_ii = 1; 
            while 1,
                pause(1);
                for kk = 0:this.num_planners_-1,
                    [config_stat(kk+1), mesh_stat(kk+1), policy_stat(kk+1), ...
                     policy_failed(kk+1)] = this.GetStatus(kk);
                end
                for kk = 0:this.num_planners_-1,
                    if done_planning, 
                        config_stat = ones(1, this.num_planners_);
                        mesh_stat = ones(1, this.num_planners_);
                    end
                    if config_stat(kk+1) == 0 && mesh_stat(kk+1) == 0, 
                        this.SetBusy(kk);
                        (ii-1)*numel(goal_point(1,:)) + (jj-1)
                        disp('Sending BVP Now');
                        success = this.SetBVP(kk, start_point(:, ii), goal_point(:, jj), ...
                                              mesh);
                        working_start_points(:, kk+1) = start_point(:, ...
                                                                    ii);
                        working_goal_points(:, kk+1) = goal_point(:, ...
                                                                  ii);
                        if jj == numel(goal_point(1,:)),
                            if ii == numel(start_point(1,:)),
                                saving_ii = ii; 
                                done_planning = true; % Stop all
                                                      % path creation
                            else
                                saving_ii = ii; 
                                ii = ii + 1;
                            end                            
                            jj = 1;
                            ready_to_save = true;
                        else
                            jj = jj+1; 
                        end
                    elseif policy_stat(kk+1) == 1, 
                        disp('Getting Spline'); 
                        [spline] = this.GetSpline(kk);
                        this.SetAvailable(kk);
                        is_found = true; 
                        this.SaveBVPSpline(working_start_points(:, ...
                                                                kk+1), ...
                        working_goal_points(:, kk+1), mesh, ...
                            spline);
                        if ready_to_save == true, 
                            disp('---------------------------');
                            disp(['SAVING POLICY ', ...
                                  num2str(saving_ii)]);
                            disp('---------------------------');
                            this.SaveAll(saving_ii);
                            ready_to_save = false;
                        end
                    elseif policy_failed(kk+1) == 1,
                        this.SetAvailable(kk);
                        disp('Didnt work...');
                        is_found = false;
                    end
                end
            end
        end
        % We should have a huge matrix of all of our solutions now

        function SaveAll(this, num_sol)
        % Save and clear the spline matrix
            filename = ['PathSolutions', num2str(num_sol), '.txt'];
            dlmwrite(filename, this.bvp_splines_, ' ');
            this.bvp_splines_ = [];
        end
        
        function [is_found] = FindSinglePolicy(this, start_point, goal_point, mesh, sim_num)
        % This function only uses the PathPlanner called Sim0, 
        % and only tests one path on one mesh. 
        % Parameters for GenMesh
            this.SetBusy(sim_num);
            is_found = false;
            while 1, 
                pause(.2);
                [config_stat, mesh_stat, policy_stat, policy_failed] = this.GetStatus(sim_num);
                if config_stat == 0 && mesh_stat == 0, 
                    disp('Sending BVP Now');
                    success = this.SetBVP(sim_num, start_point, goal_point, ...
                                          mesh);
                elseif policy_stat == 1, 
                    disp('Getting Spline'); 
                    [spline] = this.GetSpline(sim_num);
                    this.SetAvailable(sim_num);
                    is_found = true; 
                    this.SaveBVPSpline(start_point, goal_point, ...
                                       mesh, spline);
                    break;
                elseif policy_failed == 1, 
                    this.SetAvailable(sim_num);
                    disp('Didnt work...');
                    is_found = false;
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
        
        
        %%% Save our spline function to our member variable
        function SaveBVPSpline(this, start_point, goal_point, mesh, spline)
        % Each solution takes up one row in this
        % space-delimited file. The order of variables: 
        % mesh.zz (31x21), start (theta, vel), goal (x, y, th,
        % vel), spline (6 x,y coordinates in order. This should
        % make a row of 669 elements           
            BVPSolution = [];
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
            this.bvp_splines_ = [this.bvp_splines_; BVPSolution];
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
                    this.available_sims_ = [this.available_sims_, sim_num];
                end
            else
                this.busy_sims_(busy) = []; % removes them
                                            % from array
            end
        end
        
        function [is_avail] = IsAvailable(this, sim_num)
            avail = find(this.available_sims_ == sim_num);
            if isempty(avail),
                is_avail = false; 
            else
                is_avail = true;
            end
        end
        
        function [sim_num] = GetAvailableSim(this)
            if isempty(this.available_sims_), 
                sim_num = -1; 
            else
                sim_num = this.available_sims_(0);
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
            config_success = this.SetConfiguration(planner_num, start_state, ...
                                                   goal_state);
            pause(.2);
            mesh_success = this.SetHeightmap(planner_num, mesh);
            pause(.2);
            success = 0;
            if config_success == 1 && mesh_success == 1, 
                success = 1;
            end
        end
        
        function [config_status, mesh_status, policy_status, policy_failed] = GetStatus(this, planner_num)
            [config_status, mesh_status, policy_status, policy_failed] = PlannerMaster_mex('GetStatus', this.planners_, planner_num);        
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
