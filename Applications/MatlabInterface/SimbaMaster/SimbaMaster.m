%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% MATLAB class wrapper to the Node C++ architecture.
%%%% Any method with 'SimbaMaster_mex(...)'
%%%% represents a call to the Node wrapper for MATLAB.

classdef SimbaMaster < handle

    properties (SetAccess = public)
        node_; % Handle to the node instance
    end
    
    methods
        
        %%%% CONSTRUCTOR/DESTRUCTOR
        
        function this = SimbaMaster(num_sims)
            this.node_ = SimbaMaster_mex('new', num_sims);
            PlannerMaster_mex('StartConnections', this.planners_, this.num_planners_);
        end
        
        function delete(this)
            SimbaMaster_mex('delete', this.node_);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% SETTERS/GETTERS

        function SetConfiguration(this, start_state, goal_state)
            SimbaMaster_mex('SetConfiguration', this.node_, start_state, ...
                            goal_state);
        end
        
        function SetHeightmap(this, mesh)
            SimbaMaster_mex('SetHeightmap', this.node_, mesh.xx, mesh.yy, mesh.zz, ...
                            mesh.row_count, mesh.col_count);
        end            
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% INTERFACE FUNCTIONS
        %%%% All of these send with the node name "SimbaMaster"
        
        function [motion_sample] = RunPolicy(this, policy)
            [x, y, z, r, p, q] = SimbaMaster_mex('RunPolicy', ...
                                                 this.node_, ...
                                                 policy.force, ...
                                                 policy.phi, policy.duration);
            motion_sample.x = x;
            motion_sample.y = y;
            motion_sample.z = z;
            motion_sample.r = r;
            motion_sample.p = p;        
            motion_sample.q = q;
        end
        
        function PlotMotionSample(this, motion_sample)
            SimbaMaster_mex('PlotMotionSample', this.node_, ...
                            motion_sample.x, motion_sample.y, ...
                            motion_sample.z, motion_sample.r, ...
                            motion_sample.p, motion_sample.q);
        end    
        
        function SavePolicy(this, policy, mesh, start_params, goal_params)
            force = policy.force;
            phi = policy.phi;
            time = policy.time;
            meshX = mesh.xx;
            meshY = mesh.yy;
            meshZ = mesh.zz;
            start_params;
            goal_params;
            filename = ['Mesh-' num2str(tau)];
            save(filename,'force','phi', 'time', 'tau', ...
                 'start_params', 'goal_params', 'meshX', 'meshY', 'meshZ', '-append',...
                 '-ascii', '-double', '-tabs');
        end
        
        %%%% Execute a policy (force, phi, time) on a given mesh, and
        %%%% plot the results. The mesh is from GenMesh.
        
        function ExecutePolicy(this, start_state, mesh, policy)
        % For this, it doesn't matter what the goal is. 
            goal_state = [0,0,0,0,0,0];
            this.SetConfiguration(start_state, goal_state)
            this.SetHeightmap(mesh);
            [motion_sample] = this.RunPolicy(policy);
            this.PlotMotionSample(motion_sample);
        end
        
    end
end

