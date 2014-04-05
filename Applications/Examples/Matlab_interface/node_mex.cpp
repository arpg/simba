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
   *
   * SENDING/RECEIVING COMMANDS
   *
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
    bool confirm = Node_instance->SendBVP(int(*sim_num), start_point,
                                          goal_point, int(*row_count),
                                          int(*col_count), int(*tau),
                                          x_data, y_data, z_data);
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* sent = mxGetPr(plhs[0]);
    if(confirm){
      // The BVP was successfully sent, and the Sim chosen is now working hard.
      *sent = 0;
    }
    else{ *sent = 1; }
    return;
  }

  if (!strcmp("ReceiveCommands", cmd)) {
    double* sim_num = mxGetPr(prhs[2]);
    double* commands = Node_instance->ReceiveCommands(int(*sim_num));
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    // TODO: What to return here...
    //    double* ShapeIndex = mxGetPr(plhs[0]);
    //    ShapeIndex = commands;
    return;
  }

  /**************************/

  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}
