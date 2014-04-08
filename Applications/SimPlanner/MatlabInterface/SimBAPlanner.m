classdef SimBAPlanner < handle
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% MATLAB class wrapper to the Node C++ architecture.
  %%%% Any method with 'node_mex(...)'
  %%%% represents a call to the Node wrapper for MATLAB.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  properties (SetAccess = public)
    sims_;         % Handle to the node instance
    num_sims_;
    dir_to_SimPlanner_;
    goal_states_;   % Holds all possible combos of start/goal
  end
  
  methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CONSTRUCTOR/DESTRUCTOR
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function this = SimBAPlanner(num_sims)
      this.sims_ = node_mex('new', num_sims);
      % TODO: Init all possible start/goal configurations
      this.PopulateGoals();
      this.num_sims_ = num_sims;
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
      disp('[MATLAB] Starting Sim connections...');
      node_mex('StartConnections', this.sims_, this.num_sims_);
      for tau = 0:2^2,
        
        % TODO: Put scale in as part of the tau bits.
        mesh = GenMesh(tau, 3);
        cur_pol = 1;
        cur_goal_state = this.GetNextBVP(cur_pol);
        while cur_pol <  numel(this.goal_states_(1,:)),
          for ii=0:(this.num_sims_-1),
            % CheckSimStatus will return two values:
            % problem = 1: the Sim is ready to receive a problem
            % policy = 1: the Sim is ready to provide a solved policy
            disp(['[MATLAB] Checking sim status for sim ', num2str(ii)]);
            [problem, policy] = node_mex('CheckSimStatus', this.sims_, ii);
            if problem == 1,
              disp('[MATLAB] We can send a problem now!');
              start_point = [1; -.5; pi/2; 0];
              % This sends the BVP until it's passed. 
              disp('[MATLAB] start_state: ');
              start_point
              disp('[MATLAB] goal_state: ');
              cur_goal_state
              node_mex('SendBVP', this.sims_, ii, tau, mesh.xx, mesh.yy, mesh.zz, ...
                mesh.row_count, mesh.col_count, start_point, cur_goal_state);
              cur_pol = cur_pol+1;
              if cur_pol == numel(this.goal_states_(1,:)),
                break;
              end
              cur_goal_state = this.GetNextBVP(cur_pol);
            end
            
            if policy == 1,
              disp('[MATLAB] We can get a policy now!');
              [force, phi, time, start_params, goal_params] = ...
                node_mex('ReceivePolicy', this.sims_, ii);
              force
              phi
              time
              tau
              start_params
              goal_params
              this.SavePolicy(force, phi, time, tau, ...
                start_params, goal_params, mesh.X, mesh.Y, mesh.Z); 
            end
            disp(['Our current policy is', num2str(cur_goal_state)]);
          end
        end
      end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function PopulateGoals(this)
      for ii = 0:3,
        x = .5 + (ii/2);
        for jj = 0:3,
          y = .5 + (jj/2);
          for kk = 0:6,
            yaw = (-pi/2) + (pi*(kk/6));
            for ll = 1:6,
              % vel => end velocity
              vel = (2*ll);
              new_goal(1) = x;
              new_goal(2) = y;
              new_goal(3) = yaw;
              new_goal(4) = vel;
              this.goal_states_ = [this.goal_states_, new_goal'];
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
      save([filename '.mat'],'force','phi', 'time', 'tau', ...
        'start_params', 'goal_params', 'meshX', 'meshY', 'meshZ');
    end
    
  end
end
