#include "MatlabMaster.h"

//////////////////
/// CONSTRUCTOR
/// Creates all of our sims, and then assigns them all names.
//////////////////
MatlabMaster::MatlabMaster(int num_sims){
  node_.init("MATLAB");
  LOG(INFO) << "MATLAB is successfully advertizing 'BVP'";
  /// I think this should work...
  for(int i = 0; i<num_sims; i++){
    std::string sim_name = "Sim"+std::to_string(i);
    node_.advertise("BVP"+std::to_string(i));
    while(!node_.subscribe(sim_name+"/CheckNeed")){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      LOG(INFO)<<"=>";
    }
    while(!node_.subscribe(sim_name+"/CheckSolved")){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      LOG(INFO)<<"=>";
    }
    while(!node_.subscribe(sim_name+"/Policy")){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      LOG(INFO)<<"=>";
    }
    LOG(INFO)<<"MATLAB is subscribed to '"+sim_name+"/Policy'";
  }
  // Populate our goal commands.
  // Our grid goes from x(0:2), y(0:2). Keep goals between .5 and 1.5?
  double x = .5;
  double y = .5;
  double yaw = -PI/6;
  double vel = 0;
  for(int ii=0; ii<13; ii++){
    x = x + (ii/12);
    for(int jj=0; jj<13; jj++){
      y = y + (jj/12);
      for(int kk=0; kk<13; kk++){
        yaw = yaw + (kk/6);
        for(int ll=1; ll<6; ll++){
          vel = vel + (2*ll);
          std::vector<double> new_goal;
          new_goal.push_back(x);
          new_goal.push_back(y);
          new_goal.push_back(yaw);
          new_goal.push_back(vel);
          goal_states_.push_back(new_goal);
        }
      }
    }
  }
  LOG(INFO)<<"Done with goals";
}

////////////////////////
/// SETTING OUR GOAL
////////////////////////

bool MatlabMaster::CreateBVP(int tau, std::string file_name,
                          std::vector<double> goal_pt){
  double* x_data;
  double* y_data;
  double* z_data;
  int row_count;
  int col_count;
  // I: Read the file out into a form that we can use.
  MATFile *pmat;
  mxArray *pa;
  pmat = matOpen(file_name.c_str(), "r");
  // Since we already know the fields, we can match them up ourselves.
  pa = matGetVariable(pmat, "xx");
  x_data = mxGetPr(pa);
  pa = matGetVariable(pmat, "yy");
  y_data = mxGetPr(pa);
  pa = matGetVariable(pmat, "zz");
  z_data = mxGetPr(pa);
  pa = matGetVariable(pmat, "row_count");
  row_count = int(*mxGetPr(pa));
  pa = matGetVariable(pmat, "col_count");
  col_count = int(*mxGetPr(pa));
  double* start_point = new double[4];
  start_point[0] = 1;
  start_point[1] = -.5;
  start_point[2] = PI/2;
  start_point[3] = 0;
  double* goal_point = goal_pt.data();
  // I don't know how else to do this, 'cause I'm ig'nant.
  params_.Clear();
  for (int ii=0; ii<4;ii++) {
    params_.add_start_param(start_point[ii]);
    params_.add_goal_param(goal_point[ii]);
  }
  for (int ii=0;ii<(row_count*col_count);ii++) {
    params_.add_x_data(x_data[ii]);
    params_.add_y_data(y_data[ii]);
    params_.add_z_data(z_data[ii]);
  }
  params_.set_col_count(col_count);
  params_.set_row_count(row_count);
  params_.set_tau(tau);
}

////////////////////////
/// SENDING/RECEIVING COMMANDS
////////////////////////
std::vector< std::vector<double> > MatlabMaster::ReceiveCommands(int sim_num){
  std::vector< std::vector<double> > policy;
  policy.clear();  // Just to make sure that it returns empty.
  pb::RegisterBVPReqMsg req;
  pb::RegisterBVPRepMsg rep;
  node_.call_rpc("Sim"+std::to_string(sim_num), "CheckSolved", req, rep);
  bool check_need = rep.success();
  if (check_need==true) {
    while (!node_.receive("Sim"+std::to_string(sim_num)+"/Policy", policy_)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      LOG(INFO)<<"<-";
      // Keep trying!
    }
    policy = TurnPolicyIntoVector(policy_);
  }
  return policy;
}

std::vector< std::vector<double> > MatlabMaster::TurnPolicyIntoVector(
    pb::BVP_policy buffer){
  std::vector< std::vector<double> > policy;
  std::vector<double> force;
  std::vector<double> phi;
  std::vector<double> time;
  // Force
  for (int ii=0; ii<buffer.force_size(); ii++) {
    force.push_back(buffer.force(ii));
  }
  // Phi
  for (int ii=0; ii<buffer.phi_size(); ii++) {
    phi.push_back(buffer.phi(ii));
  }
  // Time
  for (int ii=0; ii<buffer.time_size(); ii++) {
    time.push_back(buffer.time(ii));
  }
  policy.push_back(force);
  policy.push_back(phi);
  policy.push_back(time);
  return policy;
}

std::vector<double> MatlabMaster::GetNextBVP(int cur_pol){
  std::vector<double> BVP = goal_states_.at(cur_pol);
  return BVP;
}

bool MatlabMaster::SavePolicyToMat(std::vector< std::vector<double> > policy,
                                int tau){
  // Command = force | pi | time
  // tau should also be saved, but it's included in the loop.
  // save a file:
  // File should also be in .mat format, if possible
  // name: Policy-%d.dat, where %d is tau
  // Should include BVP, Theta, and the tau associated.
  std::vector<double> forces = policy.at(0);
  std::vector<double> phi = policy.at(1);
  std::vector<double> time = policy.at(2);
  double* d_forces = forces.data();
  double* d_phi = phi.data();
  double* d_time = time.data();

  // The below code is a modification from an example given by MATLAB:
  // http://bit.ly/1dvUEGF

  MATFile *pmat;
  mxArray *pa_forces, *pa_phi, *pa_time;
  std::string name = "Policy-"+std::to_string(tau)+".mat";
  const char *file = name.c_str();
  pmat = matOpen(file, "w");
  int status;  // This is just for debugging purposes.
  pa_forces = mxCreateDoubleMatrix(1, forces.size(), mxREAL);
  pa_phi = mxCreateDoubleMatrix(1, phi.size(), mxREAL);
  pa_time = mxCreateDoubleMatrix(1, time.size(), mxREAL);
  memcpy((void *)(mxGetPr(pa_forces)), (void *)d_forces, sizeof(d_forces));
  memcpy((void *)(mxGetPr(pa_phi)), (void *)d_phi, sizeof(d_phi));
  memcpy((void *)(mxGetPr(pa_time)), (void *)d_time, sizeof(d_time));
  status = matPutVariable(pmat, "forces", pa_forces);
  status = matPutVariable(pmat, "phi", pa_phi);
  status = matPutVariable(pmat, "time", pa_time);
  mxDestroyArray(pa_forces);
  mxDestroyArray(pa_phi);
  mxDestroyArray(pa_time);
  matClose(pmat);
  return true;
}

/********************
 * OUR MAIN LOOP
 * *****************/

int main(int argc, char** argv){
  int num_sims = 2;
  MatlabMaster* master = new MatlabMaster(num_sims);
  int tau = 0;
  for (tau=0; tau<1; tau++) {
    // Grab the mesh based off of the number tau.
    // aka tau = 1 means we grab Mesh-1.mat.
    std::string file_name =
        "/Users/Trystan/Code/simba_tests/MatlabInterface/MeshData/Mesh-"+
        std::to_string(tau)+".mat";
    std::vector<double> cur_goal_state;
    int cur_pol = 0; //What goal we're on now.
    cur_goal_state = master->GetNextBVP(cur_pol);
    while (cur_pol<master->goal_states_.size()) {
      // Now iterate through all start/goal configs
      bool give = false;
      bool take = false;
      for (int ii=0; ii<num_sims; ii++) {
        if (cur_pol==master->goal_states_.size()) { break; }
        pb::BVP_check sim_needs_bvp;
        pb::BVP_check sim_solved;
        while (give==false && take==false) {
          // give and take should never both be true.
          std::this_thread::sleep_for(std::chrono::seconds(1));
          if(master->node_.receive("Sim"+std::to_string(ii)+"/CheckNeed",
                                   sim_needs_bvp)){
            LOG(INFO)<<"We have need!";
            give = sim_needs_bvp.need();
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
          if(master->node_.receive("Sim"+std::to_string(ii)+"/CheckSolved",
                                   sim_solved)){
            take = sim_solved.need();
          }
        }
        // If we're out of the loop, it means that Sim(ii) needs to do...
        // something.
        if (give==true) {
          LOG(INFO)<<"Starting to give new parameters now...";
          master->CreateBVP(tau, file_name, cur_goal_state);
          while (!master->node_.publish("BVP"+std::to_string(ii),
                                        master->params_)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
          cur_pol++;
          if (cur_pol==master->goal_states_.size()) {
            break;
          }
          cur_goal_state = master->GetNextBVP(cur_pol);
          give = false;
        }
        if (take==true) {
          int count = 0;
          while (!master->node_.receive("Sim"+std::to_string(ii)+
                                        "/Policy",
                                        master->policy_) && count<5) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            count++;
          }
          if(count<5){
            auto policy = master->TurnPolicyIntoVector(master->policy_);
            master->SavePolicyToMat(policy, tau);
          }
          take = false;
        }
      }
    }
  }
}
