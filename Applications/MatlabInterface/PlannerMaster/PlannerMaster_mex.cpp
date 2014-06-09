#include "mex.h"
#include "class_handle.hpp"
#include "iostream"
#include "PlannerMaster.h"

/*********************************************************************
 * CONSTRUCTOR/DESTRUCTOR OF POINTERS
 **********************************************************************/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Get the command string
  char cmd[64];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("First input should be a tring less than 64 characters long.");

  // New
  if (!strcmp("new", cmd)) {
    // Check parameters
    if (nlhs != 1)
      mexErrMsgTxt("New: One output expected.");
    PlannerMaster* node_wrap = new PlannerMaster;
    plhs[0] = convertPtr2Mat<PlannerMaster>(node_wrap);
    void mexUnlock(void);
    return;
  }

  // Check there is a second input, which should be the class instance handle
  if (nrhs < 2)
    mexErrMsgTxt("Second input should be a class instance handle.");

  // Delete
  if (!strcmp("delete", cmd)) {
    // Destroy the C++ object
    destroyObject<PlannerMaster>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2)
      mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
    return;
  }

  // Get The Class Instance Pointer From The Second Input
  PlannerMaster *planner_ptr = convertMat2Ptr<PlannerMaster>(prhs[1]);

  /*********************************************************************
   * CHECK STATUS OF SIMS
   **********************************************************************/

  if (!strcmp("StartConnections", cmd)) {
    std::cout<<"[node.mex] Checking Sim Status..."<<std::endl;
    double* num_sims = mxGetPr(prhs[2]);
    //    char dir_to_sim[80];
    //    mxGetString(prhs[3], dir_to_sim, sizeof(dir_to_sim));
    // This "a" is just a placeholder until I can get that
    // system call working.
    planner_ptr->StartConnections(int(*num_sims), "a");
    return;
  }

  if (!strcmp("CheckSimStatus", cmd)) {
    std::cout<<"[node.mex] Checking Sim Status..."<<std::endl;
    double* sim_num = mxGetPr(prhs[2]);
    double* status = planner_ptr->CheckSimStatus(int(*sim_num));
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* problem = mxGetPr(plhs[0]);
    double* policy = mxGetPr(plhs[1]);
    *problem = status[0];
    *policy = status[1];
    return;
  }

  /*********************************************************************
   * SENDING/RECEIVING COMMANDS
   **********************************************************************/

  if (!strcmp("SendBVP", cmd)) {
    // This one command both sets the command and sends it to the Sim.
    double* sim_num = mxGetPr(prhs[2]);
    double* tau = mxGetPr(prhs[3]);
    double* x_data = mxGetPr(prhs[4]);
    double* y_data = mxGetPr(prhs[5]);
    double* z_data = mxGetPr(prhs[6]);
    double* row_count = mxGetPr(prhs[7]);
    double* col_count = mxGetPr(prhs[8]);
    double* start_point = mxGetPr(prhs[9]);
    double* goal_point = mxGetPr(prhs[10]);
    planner_ptr->SendBVP(int(*sim_num), int(*tau),
                           x_data, y_data, z_data,
                           int(*row_count), int(*col_count),
                           start_point, goal_point);
    return;
  }

  ///////////

  if (!strcmp("GetPolicy", cmd)) {
    double* sim_num = mxGetPr(prhs[2]);
    double* commands = planner_ptr->ReceivePolicy(int(*sim_num));
    // The first element of 'commands' has the length of all command
    // vectors.
    plhs[0] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(4, 1, mxREAL);
    plhs[4] = mxCreateDoubleMatrix(4, 1, mxREAL);
    int counter = 1;
    double* force = mxGetPr(plhs[0]);
    double* phi = mxGetPr(plhs[1]);
    double* time = mxGetPr(plhs[2]);
    double* start_params = mxGetPr(plhs[3]);
    double* goal_params = mxGetPr(plhs[4]);
    for(int ii=0; ii<commands[0]; ii++){
      force[ii] = commands[counter];
      counter++;
    }
    for(int ii=0; ii<commands[0]; ii++){
      phi[ii] = commands[counter];
      counter++;
    }
    for(int ii=0; ii<commands[0]; ii++){
      time[ii] = commands[counter];
      counter++;
    }
    for(int ii=0; ii<4; ii++){
      start_params[ii] = commands[counter];
      counter++;
    }
    for(int ii=0; ii<4; ii++){
      goal_params[ii] = commands[counter];
      counter++;
    }
    return;
  }

  /////////////

  if (!strcmp("SendHeightmap", cmd)) {
    // This one command both sets the command and sends it to the Sim.
    double* x_data = mxGetPr(prhs[2]);
    double* y_data = mxGetPr(prhs[3]);
    double* z_data = mxGetPr(prhs[4]);
    double* row_count = mxGetPr(prhs[5]);
    double* col_count = mxGetPr(prhs[6]);
    planner_ptr->SendHeightmap(x_data, y_data, z_data,
                                 int(*row_count), int(*col_count));
    return;
  }

  /**************************/

  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}
