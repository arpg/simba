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
    if (nlhs != 1) {
      mexErrMsgTxt("New: One output expected.");
    }
    PlannerMaster* node_wrap = new PlannerMaster;
    plhs[0] = convertPtr2Mat<PlannerMaster>(node_wrap);
    void mexUnlock(void);
    return;
  }
  // Check there is a second input, which should be the class instance handle
  if (nrhs < 2) {
    mexErrMsgTxt("Second input should be a class instance handle.");
  }
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
   * NODE FUNCTIONS
   **********************************************************************/


  /*********************************************************************
   * SETTERS AND GETTERS
   **********************************************************************/

  if (!strcmp("SetConfiguration", cmd)) {
    // This one command both sets the command and sends it to the Planner.
    double* planner_num = mxGetPr(prhs[2]);
    double* start_state = mxGetPr(prhs[3]);
    double* goal_state = mxGetPr(prhs[4]);
    int success = planner_ptr->SetConfiguration(int(*planner_num),
                                                start_state, goal_state);
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* success_status = mxGetPr(plhs[0]);
    *success_status = success;
    return;
  }

  if (!strcmp("SetHeightmap", cmd)) {
    // This one command both sets the command and sends it to the Planner.
    double* planner_num = mxGetPr(prhs[2]);
    double* x_data = mxGetPr(prhs[3]);
    double* y_data = mxGetPr(prhs[4]);
    double* z_data = mxGetPr(prhs[5]);
    double* row_count = mxGetPr(prhs[6]);
    double* col_count = mxGetPr(prhs[7]);
    int success = planner_ptr->SetHeightmap(int(*planner_num),
                                            x_data, y_data, z_data,
                                            int(*row_count), int(*col_count));
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* success_status = mxGetPr(plhs[0]);
    *success_status = success;
    return;
  }

  //////////

  if (!strcmp("GetStatus", cmd)) {
    double* planner_num = mxGetPr(prhs[2]);
    double* status = planner_ptr->GetStatus(int(*planner_num));
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* config_status = mxGetPr(plhs[0]);
    double* mesh_status = mxGetPr(plhs[1]);
    double* policy_status = mxGetPr(plhs[2]);
    *config_status = status[0];
    *mesh_status = status[1];
    *policy_status = status[2];
    return;
  }

  if (!strcmp("GetPolicy", cmd)) {
    double* planner_num = mxGetPr(prhs[2]);
    double* commands = planner_ptr->GetPolicy(int(*planner_num));
    // The first element of 'commands' has the length of all command
    // vectors.
    plhs[0] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(commands[0], 1, mxREAL);
    int counter = 1;
    double* force = mxGetPr(plhs[0]);
    double* phi = mxGetPr(plhs[1]);
    double* time = mxGetPr(plhs[2]);
    for (int ii = 0; ii < commands[0]; ii++) {
      force[ii] = commands[counter];
      counter++;
    }
    for (int ii = 0; ii < commands[0]; ii++) {
      phi[ii] = commands[counter];
      counter++;
    }
    for (int ii = 0; ii<commands[0]; ii++) {
      time[ii] = commands[counter];
      counter++;
    }
    return;
  }

  if (!strcmp("GetMotionSample", cmd)) {

    // TODO

    double* planner_num = mxGetPr(prhs[2]);
    double* sample = planner_ptr->GetMotionSample(int(*planner_num));
    // plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    // plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    // double* problem = mxGetPr(plhs[0]);
    // double* policy = mxGetPr(plhs[1]);
    // *problem = status[0];
    // *policy = status[1];
    return;
  }

  if (!strcmp("GetSpline", cmd)) {
    double* planner_num = mxGetPr(prhs[2]);
    double* spline = planner_ptr->GetSpline(int(*planner_num));
    // The first element of 'commands' has the length of all command
    // vectors.
    plhs[0] = mxCreateDoubleMatrix(spline[0], 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(spline[0], 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* x_values = mxGetPr(plhs[0]);
    double* y_values = mxGetPr(plhs[1]);
    double* solved_goal_pose = mxGetPr(plhs[2]);
    int counter = 1;
    for (int ii = 0; ii < spline[0]; ii++) {
      x_values[ii] = spline[counter];
      counter++;
    }
    for (int ii = 0; ii < spline[0]; ii++) {
      y_values[ii] = spline[counter];
      counter++;
    }
    for (int ii = 0; ii<4; ii++) {
      solved_goal_pose[ii] = spline[counter];
      counter++;
    }
    return;
  }

  /**************************/

  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}
