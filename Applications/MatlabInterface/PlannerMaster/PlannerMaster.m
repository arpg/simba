% PLANNER MASTER
%   MATLAB class wrapper to the Node C++ architecture.
%   This MATLAB program works simultaneously with the program
%   PathPlanner (./Applications/PathPlanner) to solve Boundary
%   Value Problems (BVP). 

classdef PlannerMaster < handle
    
    properties (SetAccess = public)
        planners_;         % Handle to the node instance
        num_planners_;
        goal_states_;   % Holds all possible combos of start/goal
        cur_pol_;
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% CONSTRUCTOR/DESTRUCTOR
        
        function this = PlannerMaster(planner_nums)
            this.planners_ = PlannerMaster_mex('new', num_planners);
            % TODO: Init all possible start/goal configurations
            this.PopulateGoals();
            this.num_planners_ = num_planners;
            this.cur_pol_ = 1;
        end
        
        function delete(this)
            PlannerMaster_mex('delete', this.planners_);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% CREATE OUR LOOKUP TABLE
        %%%% This function uses all possible 18-bit combinations to generate
        %%%% a series of small heightmaps. Each of these heightmaps is passed
        %%%% to a series of simulator instances, which produce commands for a
        %%%% specified set of start/goal configurations.

        function FindPolicies(this)
        % Since we have a ton of meshes (2^18, to be exact)...
        % start with 4 just to be safe.
            this.ConnectNode();
            % Parameters for GenMesh
            granularity = 15;
            scale = 1;      
            for tau = 0:0,  
                mesh = GenMesh(tau, granularity, scale);
                this.cur_pol_ = 1;
                cur_goal_state = this.GetNextBVP(this.cur_pol_);
                while this.cur_pol_ <=  numel(this.goal_states_(1,:)),
                    for ii = 0:(this.num_planners_-1),
                        [problem, policy] = this.GetStatus(ii);
                        if problem == 1,
                            start_point = [1; -.5; 0; 0];
                            start_point(1:2) = start_point(1:2) * scale;
                            cur_goal_state(1:2) = cur_goal_state(1:2) * scale;
                            % This sends the BVP until it's passed. 
                            this.SetBVP(ii, start_point, cur_goal_state, mesh);
                            this.cur_pol_ = this.cur_pol_+1;
                            if this.cur_pol_ == numel(this.goal_states_(1,:)),
                                break;
                            end
                            cur_goal_state = this.GetNextBVP(this.cur_pol_);
                        end
                        
                        if policy == 1,
                            [force, phi, time, start_params, goal_params] = ...
                                this.GetPolicy(ii);
                            this.SavePolicy(force, phi, time, tau, ...
                                            start_params, goal_params, ...
                                            mesh.X, mesh.Y, mesh.Z); 
                        end
                        disp('Our current goal state is');
                        cur_goal_state
                    end
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% MATLAB FUNCTIONS
        %%%% These don't interact with node at all. 
        
        function PopulateGoals(this)
        %       for ii = 0:3,
        %         x = .5 + (ii/2);
        %         for jj = 0:3,
        %           y = .5 + (jj/2);
        %           for kk = 0:6,
        %             % We have to compensate for the way I stretched the mesh
        %             % ...I can fix it later. 
        %             yaw = (-pi/2) + (pi*(kk/6));
        %             for ll = 1:6,
        %               % vel => end velocity
        %               vel = (2*ll);
        %               new_goal(1) = x;
        %               new_goal(2) = y;
        %               new_goal(3) = yaw;
        %               new_goal(4) = vel;
            this.goal_states_ = [1; 1.5; 0; 1]; 
            %               this.goal_states_ = [this.goal_states_, new_goal'];
            %             end
            %           end
            %         end        
            %       end
        end

        function BVP = GetNextBVP(this, cur_pol)
            BVP = this.goal_states_(:, cur_pol);
        end
        
        %%% Save our Policy to a .mat file
        function SavePolicy(this, force, phi, time, tau, ...
                            start_params, goal_params, meshX, meshY, meshZ)
            force;
            phi;
            time;
            meshX;
            meshY;
            meshZ;
            start_params;
            goal_params;
            filename = ['Mesh-' num2str(tau)];
            save(filename,'force','phi', 'time', 'tau', ...
                 'start_params', 'goal_params', 'meshX', 'meshY', 'meshZ', '-append',...
                 '-ascii', '-double', '-tabs');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% NODE FUNCTIONS
        
        function ConnectNode(this)
            PlannerMaster_mex('ConnectNode', this.planners_, this.num_planners_);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% SETTERS AND GETTERS
        %%%% All of these send with the node name "MATLAB"
        
        function SetConfiguration(this, planner_num, start_state, ...
                                  goal_state)
            PlannerMaster_mex('SetConfiguration', this.planners_, planner_num, start_state, ...
                              goal_state);
        end
        
        function SetHeightmap(this, planner_num, mesh)
            PlannerMaster_mex('SetHeightmap', this.planners_, ...
                              planner_num, mesh.xx, mesh.yy, mesh.zz, ...
                              mesh.row_count, mesh.col_count);
        end    
        
        function SetBVP(this, planner_num, start_state, goal_state, mesh)
            this.SetConfiguration(planner_num, start_state, ...
                                  goal_state);
            this.SetHeightmap(planner_num, mesh);
        end
        
        function [need_problem, need_policy] = GetStatus(this, planner_num)
            [need_problem, need_policy] = PlannerMaster_mex('GetStatus', this.planners_, ii);        
        end
        
        function [policy] = GetPolicy(this, planner_num)
            [force, phi, time, start_params, goal_params] = ...
                PlannerMaster_mex('GetPolicy', this.planners_, planner_num);
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
            [a, b, c, d, e] = PlannerMaster_mex('GetSpline', this.planners_, ...
                                                planner_num);
            spline.a = a;
            spline.b = b;
            spline.c = c;
            spline.d = d;
            spline.e = e;
        end        
        
    end
end
