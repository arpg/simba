#include "mex.h"
#include "class_handle.hpp"
#include "iostream"
#include "node_wrapper.h"

/*********************************************************************
 *
 *CONSTRUCTION AND DESTRUCTION OF BULLET POINTERS
 *
 **********************************************************************/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  std::cout<<"WOOOOOO"<<std::endl;
  // Get the command string
  char cmd[64];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("First input should be a command string less than 64 characters long.");
  std::cout<<"WOOOOOO"<<std::endl;

  // New
  if (!strcmp("new", cmd)) {
    // Check parameters
    if (nlhs != 1)
      mexErrMsgTxt("New: One output expected.");
    std::cout<<"WOOOOOO"<<std::endl;
    double* num_sims = mxGetPr(prhs[2]);
    char dir_to_sim[80];
    mxGetString(prhs[3], dir_to_sim, sizeof(dir_to_sim));
    // Return a handle to a new C++ instance
    std::cout<<"WOOOOOO"<<std::endl;
    NodeWrapper* node_wrap = new NodeWrapper(int(*num_sims), dir_to_sim);
    plhs[0] = convertPtr2Mat<NodeWrapper>(node_wrap);
    void mexUnlock(void);
    return;
  }

  // Check there is a second input, which should be the class instance handle
  if (nrhs < 2)
    mexErrMsgTxt("Second input should be a class instance handle.");

  // Delete
  if (!strcmp("delete", cmd)) {
    // Destroy the C++ object
    destroyObject<NodeWrapper>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2)
      mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
    return;
  }

  // Get the class instance pointer from the second input
  NodeWrapper *Node_instance = convertMat2Ptr<NodeWrapper>(prhs[1]);

  /*********************************************************************
   * CHECK STATUS OF SIMS
   **********************************************************************/

  if (!strcmp("CheckSimStatus", cmd)) {
    double* sim_num = mxGetPr(prhs[2]);
    double* status = Node_instance->CheckSimStatus(int(*sim_num));
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* problem = mxGetPr(plhs[0]);
    double* policy = mxGetPr(plhs[1]);
    problem = status[0];
    policy = status[1];
    return;
  }

  /*********************************************************************
   * SENDING/RECEIVING COMMANDS
   **********************************************************************/

  if (!strcmp("SendBVP", cmd)) {
    // This one command both sets the command and sends it to the Sim.
    double* sim_num = mxGetPr(prhs[2]);
    double* start_point = mxGetPr(prhs[3]);
    double* goal_point = mxGetPr(prhs[4]);
    double* row_count = mxGetPr(prhs[5]);
    double* col_count = mxGetPr(prhs[6]);
    double* tau = mxGetPr(prhs[7]);
    double* x_data = mxGetPr(prhs[8]);
    double* y_data = mxGetPr(prhs[9]);
    double* z_data = mxGetPr(prhs[10]);
    Node_instance->SendBVP(int(*sim_num), int(*tau),
                           x_data, y_data, z_data,
                           int(*row_count), int(*col_count),
                           start_point, goal_point);
    return;
  }

  ///////////

  if (!strcmp("ReceivePolicy", cmd)) {
    double* sim_num = mxGetPr(prhs[2]);
    double* commands = Node_instance->ReceiveCommands(int(*sim_num));
    // The first element of 'commands' has the length of all command
    // vectors.
    plhs[0] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    int counter = 1;
    double* force = mxGetPr(plhs[0]);
    double* phi = mxGetPr(plhs[1]);
    double* time = mxGetPr(plhs[2]);
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
    return;
  }

  /**************************/

  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}
