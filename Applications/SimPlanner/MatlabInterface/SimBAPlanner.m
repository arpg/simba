classdef SimBAPlanner < handle
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% MATLAB class wrapper to the Node C++ architecture.
  %%%% Any method with 'node_mex(...)'
  %%%% represents a call to the Node wrapper for MATLAB.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  properties (SetAccess = public)
    sims_;         % Handle to the node instance
    num_sims_;
    goal_states_;   % Holds all possible combos of start/goal
  end
  
  methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CONSTRUCTOR/DESTRUCTOR
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function this = SimBAPlanner(num_sims, dir_to_SimPlanner)
      this.sims_ = node_mex('new', num_sims, dir_to_SimPlanner);
      % TODO: Init all possible start/goal configurations
      this.PopulateGoals();
      this.num_sims_ = num_sims;
      this.goal_states_ = [];
    end
    
    function delete(this)
      node_mex('delete', this.sims_);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CREATE OUR LOOKUP TABLE
    %%%% This function uses all possible 18-bit combinations to generate
    %%%% a series of small heightmaps. Each of these heightmaps is passed
    %%%% to a series of simulator instances, which produce commands for a
    %%%% specified set of start/goal configurations.
    %%%% It's a big operation.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function Run(this)
      % Since we have a ton of meshes (2^18, to be exact)...
      % start with 4 just to be safe.
      for tau = 0:2^2,
        
        % TODO: Put scale in as part of the tau bits.
        mesh = GenMesh(tau, 3);
        cur_pol = 1;
        cur_goal_state = GetNextBVP(cur_pol);
        while cur_pol <  numel(this.goal_states_(1,:)),
          for ii=0:this.num_sims_,
            % CheckSimStatus will return two values:
            % problem = 1: the Sim is ready to receive a problem
            % policy = 1: the Sim is ready to provide a solved policy
            [problem, policy] = node_mex('CheckSimStatus', this.sims_, ii);
            if problem == 1,
              start_point = [1; -.5; pi/2; 0];
              % This sends the BVP until it's passed. 
              node_mex('SendBVP', this.sims_, ii, tau, mesh.xx, mesh.yy, mesh.zz, ...
                mesh.row_count, mesh.col_count, start_point, cur_goal_state);
              cur_pol = cur_pol+1;
              if cur_pol == numel(this.goal_states_(1,:)),
                break;
              end
              cur_goal_state = GetNextBVP(cur_pol);
            end
            
            if policy == 1,
              [force, phi, time] = node_mex('ReceivePolicy', this.sims_, ii);
              SavePolicy(force, phi, time, tau, mesh); 
            end
          end
        end
      end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function PopulateGoals(this)
      x = .5;
      y = .5;
      yaw = -pi/6;
      vel = 0;
      for ii = 0:13,
        x = x + (ii/12);
        for jj = 0:13,
          y = y + (jj/12);
          for kk = 0:13,
            yaw = yaw + (kk/6);
            for ll = 1:6,
              % vel => end velocity
              vel = vel + (2*ll);
              new_goal(1) = x;
              new_goal(2) = y;
              new_goal(3) = yaw;
              new_goal(4) = vel;
              this.goal_states_ = [this.goal_states_, new_goal];
            end
          end
        end
      end
    end
    
    %%%% Get the next start/goal configuration for the next simulation.
    
    function BVP = GetNextBVP(this, cur_pol)
      BVP = this.goal_states_(:, cur_pol);
    end
    
    %%% Save our Policy
    
    function SavePolicy(force, phi, time, tau, mesh)
      mesh_x = mesh.X;
      mesh_y = mesh.Y;
      mesh_z = mesh.Z;
      filename = ['Mesh-' num2str(tau)];
      save([filename '.mat'],'force','phi', 'time', 'tau', 'mesh_x', ...
        'mesh_y', 'mesh_z');
    end
    
  end
end
