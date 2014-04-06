classdef SimBAPlanner < handle
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% MATLAB class wrapper to the Node C++ architecture.
  %%%% Any method with 'node_mex(...)'
  %%%% represents a call to the Node wrapper for MATLAB.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  properties (SetAccess = public)
    sims;         % Handle to the node instance
    num_sims;
    s_g_config;   % Holds all possible combos of start/goal
    cur_pol;      % gives the current index of s_g_config for the simulation
    bvp;          % Our current boundary value problems. For now, the car 
                  % starts at (1, -.5, pi/2); the only thing that changes
                  % is the starting velocity. 
                  % This is an array, so that we can keep track of 
                  % what has what.
  end
  
  methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CONSTRUCTOR/DESTRUCTOR
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function this = SimBAPlanner(num_sims, dir_to_SimPlanner)
      this.sims = node_mex('new', num_sims, dir_to_SimPlanner);
      % TODO: Init all possible start/goal configurations
      this.num_sims = num_sims;
      this.s_g_config = [];
      this.cur_pol = 1;
      GetNextBVP(this.cur_pol);
    end
    
    function delete(this)
      node_mex('delete', this.sims);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CREATE OUR LOOKUP TABLE
    %%%% This function uses all possible 18-bit combinations to generate
    %%%% a series of small heightmaps. Each of these heightmaps is passed
    %%%% to a series of simulator instances, which produce commands for a
    %%%% specified set of start/goal configurations.
    %%%% It's a big operation.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
%     int main(int argc, char** argv){
%   int num_sims = 2;
%   std::string dir_to_sim =
%       "/Users/Trystan/Code/simba/build/Applications/SimPlanner/SimPlanner";
%   CPPMaster* master = new CPPMaster(num_sims, dir_to_sim);
%   int tau = 0;
%   // TODO: Since we have a ton of meshes (2^18, to be exact)...
%   // start with 4 just to be safe.
%   for (tau=0; tau<(2^2 - 1); tau++) {
%     // Grab the mesh based off of the number tau.
%     // aka tau = 1 means we grab Mesh-1.mat.
%     std::string file_name =
%         "/Users/Trystan/Code/simba_tests/MatlabInterface/MeshData/Mesh-"+
%         std::to_string(tau)+".mat";
%     std::vector<double> cur_goal_state;
%     int cur_pol = 0; //What goal we're on now.
%     cur_goal_state = master->GetNextBVP(cur_pol);
%     while (cur_pol<master->goal_states_.size()) {
%       // Now iterate through all start/goal configs
%       bool give = false;
%       bool take = false;
%       for (int ii=0; ii<num_sims; ii++) {
%         if (cur_pol==master->goal_states_.size()) { break; }
%         pb::BVP_check sim_needs_bvp;
%         pb::BVP_check sim_solved;
%         while (give==false && take==false) {
%           // give and take should never both be true.
%           std::this_thread::sleep_for(std::chrono::seconds(1));
%           if(master->node_.receive("Sim"+std::to_string(ii)+"/CheckNeed",
%                                    sim_needs_bvp)){
%             cout<<"We have need!"<<endl;
%             give = sim_needs_bvp.need();
%           }
%           std::this_thread::sleep_for(std::chrono::seconds(1));
%           if(master->node_.receive("Sim"+std::to_string(ii)+"/CheckSolved",
%                                    sim_solved)){
%             take = sim_solved.need();
%           }
%         }
%         // If we're out of the loop, it means that Sim(ii) needs to do...
%         // something.
%         if (give==true) {
%           cout<<"Starting to give new parameters now..."<<endl;
%           master->CreateBVP(tau, file_name, cur_goal_state);
%           while (!master->node_.publish("BVP"+std::to_string(ii),
%                                         master->params_)) {
%             std::this_thread::sleep_for(std::chrono::seconds(1));
%           }
%           cur_pol++;
%           if (cur_pol==master->goal_states_.size()) {
%             break;
%           }
%           cur_goal_state = master->GetNextBVP(cur_pol);
%           give = false;
%         }
%         if (take==true) {
%           int count = 0;
%           while (!master->node_.receive("Sim"+std::to_string(ii)+
%                                         "/Policy",
%                                         master->policy_) && count<5) {
%             std::this_thread::sleep_for(std::chrono::seconds(1));
%             count++;
%           }
%           if(count<5){
%             auto policy = master->TurnPolicyIntoVector(master->policy_);
%             master->SavePolicyToMat(policy, tau);
%           }
%           take = false;
%         }
%       }
%     }
%   }
% }
% 
%     
    
    
    
    
    
    function Run(this)
      % Since we have a ton of meshes (2^18, to be exact)...
      % start with 4 just to be safe.
      for tau = 0:2^2,
        
        % TODO: Put scale in as part of the tau bits. 
        mesh = GenMesh(tau, 3);
        
        % Now iterate through all start/goal configs
        while this.cur_pol<numel(this.s_g_config(1, :)),
          % Check to see which Sims are available.          
          for ii = 1:numel(this.num_sims),
            sent = node_mex('SendBVP', this.sims, ii, this.bvp.start,...
              this.bvp.goal, size(mesh.xx), size(mesh.yy), tau, mesh.xx,...
              mesh.yy, mesh.zz);
            if sent==0,
              % The Sim picked up the BVP. Move to the next one. 
              GetNextBVP(this.cur_pol);
            end
          end
          % Check to see which Sims are ready with their paths 
          for ii = 1:numel(this.num_sims),
            [received, force, phi, time, this_tau] = node_mex(...
              'ReceiveCommands', this.sims, ii);
            if received == 0,
              % All theta variables should be rows of the same length
              theta.force = force;
              theta.phi = phi;
              theta.time = time;
              theta.tau = this_tau;
              % save a file: (Mesh-%d.dat, tau)
              % filename - sprintf('Mesh-%d.dat', Tau)
              Savetheta(bvp, theta, mesh.Z);
            end
          end
        end
      end  
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Get the next start/goal configuration for the next simulation.
    
    function GetNextBVP(this)
      this.bvp = this.s_g_config(:, this.cur_pol);
    end
    
    function SaveTheta(saved_bvp, saved_theta, saved_mesh_heights)
      % I'm not sure how the workspace scope works here, so this is 
      % what will do for now. 
      saved_bvp;
      saved_theta; 
      saved_mesh_heights;
      filename = ['Mesh-' num2str(saved_theta.tau)];
      save([filename '.mat'],'saved_bvp','saved_theta', 'saved_mesh_heights');
    end
    
  end
end
