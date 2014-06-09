#include "mex.h"
#include "class_handle.hpp"
#include "iostream"
#include "SimbaMaster.h"

/*********************************************************************
 * CONSTRUCTOR/DESTRUCTOR OF CLASS POINTER
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
    SimbaMaster* node_wrap = new SimbaMaster;
    plhs[0] = convertPtr2Mat<SimbaMaster>(node_wrap);
    void mexUnlock(void);
    return;
  }

  // Check there is a second input, which should be the class instance handle
  if (nrhs < 2)
    mexErrMsgTxt("Second input should be a class instance handle.");

  // Delete
  if (!strcmp("delete", cmd)) {
    // Destroy the C++ object
    destroyObject<SimbaMaster>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2)
      mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
    return;
  }

  // Get The Class Instance Pointer From The Second Input
  SimbaMaster *simba_ptr = convertMat2Ptr<SimbaMaster>(prhs[1]);

  /*********************************************************************
   * NODE FUNCTIONS
   **********************************************************************/

  if (!strcmp("StartConnections", cmd)) {
    simba_ptr->StartConnections();
    return;
  }

  /*********************************************************************
   * SETTERS/GETTERS
   **********************************************************************/

  if (!strcmp("SetConfiguration", cmd)) {
    // This one command both sets the command and sends it to the Sim.
    double* start_point = mxGetPr(prhs[2]);
    double* goal_point = mxGetPr(prhs[3]);
    simba_ptr->SetConfiguration(start_point, goal_point);
    return;
  }

  ///////

  if (!strcmp("SetHeightmap", cmd)) {
    // This one command both sets the command and sends it to the Sim.
    double* x_data = mxGetPr(prhs[2]);
    double* y_data = mxGetPr(prhs[3]);
    double* z_data = mxGetPr(prhs[4]);
    double* row_count = mxGetPr(prhs[5]);
    double* col_count = mxGetPr(prhs[6]);
    simba_ptr->SetHeightmap(x_data, y_data, z_data,
                            int(*row_count), int(*col_count));
    return;
  }

  /*********************************************************************
   * INTERFACE FUNCTIONS
   **********************************************************************/

  if (!strcmp("RunPolicy", cmd)) {
    // This creates a hal::car controller and sends all the commands in
    // its buffer (whatever we gave it)
    double* force = mxGetPr(prhs[2]);
    double* phi = mxGetPr(prhs[3]);
    double* duration = mxGetPr(prhs[4]);
    // std::vector< std::vector<double> > motion_sample =
    simba_ptr->RunPolicy(force, phi, duration);
    //// TODO: RETURN MOTION SAMPLE
    for(int ii=0; ii<6; ii++){
      plhs[ii] = mxCreateDoubleMatrix(1, 1, mxREAL);
    }
    double* x = mxGetPr(plhs[0]);
    double* y = mxGetPr(plhs[1]);
    double* z = mxGetPr(plhs[2]);
    double* r = mxGetPr(plhs[3]);
    double* p = mxGetPr(plhs[4]);
    double* q = mxGetPr(plhs[5]);
    *x = 0;
    *y = 0;
    *z = 0;
    *r = 0;
    *p = 0;
    *q = 0;
    return;
  }

  ///////

  if (!strcmp("PlotMotionSample", cmd)) {
    // Puts the motion sample calculated into the SceneGraph
    double* x = mxGetPr(prhs[2]);
    double* y = mxGetPr(prhs[3]);
    double* z = mxGetPr(prhs[4]);
    double* r = mxGetPr(prhs[5]);
    double* p = mxGetPr(prhs[6]);
    double* q = mxGetPr(prhs[7]);
    simba_ptr->PlotMotionSample(x, y, z, r, p, q);
    return;
  }

  /**************************/

  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}
