
/*********************************************************************
 * File:   bullet_interface_mex.cpp
 * Author: bminortx
 * The MEX interface that connects Bullet functions with MATLAB calls
 **********************************************************************/

#include "mex.h"
#include "class_handle.hpp"
#include "iostream"
#include "matlab_bullet_wrapper.h"
#include "boost/shared_ptr.hpp"

/*********************************************************************
 *
 *CONSTRUCTION AND DESTRUCTION OF BULLET POINTERS
 *
 **********************************************************************/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Get the command string
  char cmd[64];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("First input should be a command string less than 64 characters long.");

  // New
  if (!strcmp("new", cmd)) {
    // Check parameters
    if (nlhs != 1)
      mexErrMsgTxt("New: One output expected.");
    // Return a handle to a new C++ instance
    plhs[0] = convertPtr2Mat<Sim>(new Sim);
    void mexUnlock(void);
    return;
  }

  // Check there is a second input, which should be the class instance handle
  if (nrhs < 2)
    mexErrMsgTxt("Second input should be a class instance handle.");

  // Delete
  if (!strcmp("delete", cmd)) {
    // Destroy the C++ object
    destroyObject<Sim>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2)
      mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
    return;
  }

  // Get the class instance pointer from the second input
  Sim *Sim_instance = convertMat2Ptr<Sim>(prhs[1]);

  /*********************************************************************
   *
   *ADDING OBJECTS
   *
   **********************************************************************/

  // AddTerrain
  if (!strcmp("AddTerrain", cmd)) {
    double* row_count = mxGetPr(prhs[2]);
    double* col_count = mxGetPr(prhs[3]);
    double* grad = mxGetPr(prhs[4]);
    double* min_ht = mxGetPr(prhs[5]);
    double* max_ht = mxGetPr(prhs[6]);
    double* X = mxGetPr(prhs[7]);
    double* Y = mxGetPr(prhs[8]);
    double* Z = mxGetPr(prhs[9]);
    double* normal = mxGetPr(prhs[10]);
    int id = Sim_instance->AddTerrain(int(*row_count), int(*col_count),
                                      *grad, *min_ht, *max_ht, X, Y, Z, normal);
    double d_id = (double)id;
    //Return the index, so that we can look up the position later.
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* index = mxGetPr(plhs[0]);
    index = &d_id;
    return;
  }

  ////////////////////////////

  // AddShape
  if (!strcmp("AddShape", cmd)) {
    //Get the type of the shape
    char shape[64];
    int index;
    mxGetString(prhs[2], shape, sizeof(shape));

    ///Cube
    if(!strcmp(shape, "Cube")){
      double* width = mxGetPr(prhs[3]);
      double* length = mxGetPr(prhs[4]);
      double* height = mxGetPr(prhs[5]);
      double* mass = mxGetPr(prhs[6]);
      double* restitution = mxGetPr(prhs[7]);
      double* position = mxGetPr(prhs[8]);
      double* rotation = mxGetPr(prhs[9]);
      plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
      double* ShapeIndex = mxGetPr(plhs[0]);
      int index = Sim_instance->AddCube(*width, *length, *height, *mass,
                                        *restitution, position,
                                        rotation);
      *ShapeIndex = (double)index;
    }

    ///Sphere
    if (!strcmp(shape, "Sphere")) {
      double* radius = mxGetPr(prhs[3]);
      double* mass = mxGetPr(prhs[4]);
      double* restitution = mxGetPr(prhs[5]);
      double* position = mxGetPr(prhs[6]);
      double* rotation = mxGetPr(prhs[7]);
      plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
      double* ShapeIndex = mxGetPr(plhs[0]);
      int index = Sim_instance->AddSphere(*radius, *mass, *restitution,
                                        position, rotation);
      *ShapeIndex = (double)index;
    }

    ///Cylinder
    if (!strcmp(shape, "Cylinder")) {
      double* radius = mxGetPr(prhs[3]);
      double* height = mxGetPr(prhs[4]);
      double* mass = mxGetPr(prhs[5]);
      double* restitution = mxGetPr(prhs[6]);
      double* position = mxGetPr(prhs[7]);
      double* rotation = mxGetPr(prhs[8]);
      plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
      double* ShapeIndex = mxGetPr(plhs[0]);
      int index = Sim_instance->AddCylinder(*radius, *height, *mass,
                                            *restitution,
                                            position, rotation);
      *ShapeIndex = (double)index;
    }
    return;
  }

  //////////////////////////////////////////////

  // AddCompounds
  if (!strcmp("AddCompound", cmd)) {
    char compound_type[64];
    mxGetString(prhs[2], compound_type, sizeof(compound_type));
    if (!strcmp("Vehicle", compound_type)){
      double* Shape_ids = mxGetPr(prhs[3]);
      double* Con_ids = mxGetPr(prhs[4]);
      int index = Sim_instance->AddCompound(Shape_ids, Con_ids, compound_type);
      plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
      double* CompoundIndex = mxGetPr(plhs[0]);
      *CompoundIndex = (double)index;
    }
    return;
  }

  // AddRaycastVehicle
  if (!strcmp("AddRaycastVehicle", cmd)) {
    double* parameters = mxGetPr(prhs[2]);
    double* position = mxGetPr(prhs[3]);
    double* rotation = mxGetPr(prhs[4]);
    int index = Sim_instance->AddRaycastVehicle(parameters,
                                                position, rotation);
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* CompoundIndex = mxGetPr(plhs[0]);
    *CompoundIndex = (double)index;
    return;
  }

  /*********************************************************************
   *
   *RUNNING THE SIMULATION
   *
   **********************************************************************/

  // StepSimulation
  if (!strcmp("StepSimulation", cmd)) {
    // Check parameters
    if (nlhs < 0 || nrhs > 2 )
      mexErrMsgTxt("StepSimulation: Unexpected arguments.");
    // Call the method
    Sim_instance->StepSimulation();
    return;
  }


  /*********************************************************************
   *
   *COMPOUND METHODS
   *
   **********************************************************************/

  if (!strcmp("CommandCompound", cmd)) {
    //Get the compound type
    char compound_type[64];
    mxGetString(prhs[2], compound_type, sizeof(compound_type));
    if (!strcmp("Vehicle", compound_type)){
      double* id = mxGetPr(prhs[3]);
      double* phi = mxGetPr(prhs[4]);
      double* force = mxGetPr(prhs[5]);
      Sim_instance->CommandVehicle(*id, *phi, *force);
    }
    return;
  }

  /*********************************************************************
   *
   *RAYCAST VEHICLE METHODS
   *
   **********************************************************************/

  if (!strcmp("CommandRaycastVehicle", cmd)){
    double* id = mxGetPr(prhs[2]);
    double* phi = mxGetPr(prhs[3]);
    double* force = mxGetPr(prhs[4]);
    Sim_instance->CommandRaycastVehicle(*id, *phi, *force);
    return;
  }

  ///////

  if (!strcmp("GetMotionState", cmd)){
    double* id = mxGetPr(prhs[2]);
    double* state = Sim_instance->GetRaycastMotionState(*id);
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, 3, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, 3, mxREAL);
    plhs[4] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* steering_angle = mxGetPr(plhs[0]);
    double* force = mxGetPr(plhs[1]);
    double* lin_velocity = mxGetPr(plhs[2]);
    double* ang_velocity = mxGetPr(plhs[3]);
    double* grounded = mxGetPr(plhs[4]);
    *steering_angle = state[0];
    *force = state[1];
    lin_velocity[0] = state[2];
    lin_velocity[1] = state[3];
    lin_velocity[2] = state[4];
    ang_velocity[0] = state[5];
    ang_velocity[1] = state[6];
    ang_velocity[2] = state[7];
    grounded[0] = state[8];
    return;
  }

  ///////

  if (!strcmp("SetToGround", cmd)){
    double* id = mxGetPr(prhs[2]);
    double* x = mxGetPr(prhs[3]);
    double* y = mxGetPr(prhs[4]);
    plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
    double* position = mxGetPr(plhs[0]);
    double* pos = Sim_instance->RaycastToGround(*id, *x, *y);
    position[0] = pos[0];
    position[1] = pos[1];
    position[2] = pos[2];
    return;
  }

  ///////

  if(!strcmp("SpeedSim", cmd)){
    double* id = mxGetPr(prhs[2]);
    double* start_pose = mxGetPr(prhs[3]);
    double* start_rot = mxGetPr(prhs[4]);
    double* start_lin_vel = mxGetPr(prhs[5]);
    double* start_ang_vel = mxGetPr(prhs[6]);
    double* forces = mxGetPr(prhs[7]);
    double* steering_angles = mxGetPr(prhs[8]);
    double* command_length = mxGetPr(prhs[9]);
    plhs[0] = mxCreateDoubleMatrix(3, *command_length, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 3, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(3, 3, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, 3, mxREAL);
    plhs[4] = mxCreateDoubleMatrix(1, 3, mxREAL);
    plhs[5] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* states = Sim_instance->SpeedSim(*id, start_pose, start_rot,
                                            start_lin_vel, start_ang_vel,
                                            forces, steering_angles,
                                            *command_length);
    double* xy = mxGetPr(plhs[0]);
    double* end_pos = mxGetPr(plhs[1]);
    double* end_rot = mxGetPr(plhs[2]);
    double* end_lin_vel = mxGetPr(plhs[3]);
    double* end_ang_vel = mxGetPr(plhs[4]);
    double* grounded = mxGetPr(plhs[5]);
    int limit = 3*(*command_length);
    for(int i = 0; i < limit; i++){
      xy[i] = states[i];
    }
    end_pos[0]=states[limit];
    end_pos[1]=states[limit+1];
    end_pos[2]=states[limit+2];
    end_rot[0]=states[limit+3];
    end_rot[1]=states[limit+4];
    end_rot[2]=states[limit+5];
    end_rot[3]=states[limit+6];
    end_rot[4]=states[limit+7];
    end_rot[5]=states[limit+8];
    end_rot[6]=states[limit+9];
    end_rot[7]=states[limit+10];
    end_rot[8]=states[limit+11];
    end_lin_vel[0]=states[limit+12];
    end_lin_vel[1]=states[limit+13];
    end_lin_vel[2]=states[limit+14];
    end_ang_vel[0]=states[limit+15];
    end_ang_vel[1]=states[limit+16];
    end_ang_vel[2]=states[limit+17];
    grounded[0]=states[limit+18];
    return;
  }

  ///////

  if (!strcmp("ResetVehicle", cmd)){
    double* id = mxGetPr(prhs[2]);
    double* start_pose = mxGetPr(prhs[3]);
    double* start_rot = mxGetPr(prhs[4]);
    Sim_instance->ResetVehicle(*id, start_pose, start_rot);
    return;
  }

  /*********************************************************************
   *
   *CONSTRAINT METHODS
   *There are too many constructors for our purposes, so we're going to
   *have to put them all here
   * 1. Point-to-point constraints: One and Two-shape constraint
   * 2. Hinge: One and Two-shape constraint, both with options for a
   *    transformation on the shape, or just a pivot and axis selection.
   * 3. Hinge2 - Used for vehicle axes
   * 4. Six degrees of freedom - customizable constraint
   *
   **********************************************************************/

  //AddConstraint
  if (!strcmp("AddConstraint", cmd)) {
    //Get the type of the constraint
    char constraint[64];
    int index;
    mxGetString(prhs[2], constraint, sizeof(constraint));

    ///////
    //Point To Point
    ///////
    if (!strcmp("PointToPoint_one", constraint)){
      double* id = mxGetPr(prhs[3]);
      double* pivot_in_A = mxGetPr(prhs[4]);
      index = Sim_instance->PointToPoint_one(*id, pivot_in_A);
    }

    if (!strcmp("PointToPoint_two", constraint)){
      double* id_A = mxGetPr(prhs[3]);
      double* id_B = mxGetPr(prhs[4]);
      double* pivot_in_A = mxGetPr(prhs[5]);
      double* pivot_in_B = mxGetPr(prhs[6]);
      index = Sim_instance->PointToPoint_two(*id_A, *id_B,
                                  pivot_in_A, pivot_in_B);
    }

    ///////
    //Hinge
    ///////
    if (!strcmp("Hinge_one_transform", constraint)) {
      double* id = mxGetPr(prhs[3]);
      double* transform_A = mxGetPr(prhs[4]);
      double* limits = mxGetPr(prhs[5]);
      index = Sim_instance->Hinge_one_transform(*id, transform_A, limits);
    }

    if (!strcmp("Hinge_two_transform", constraint)) {
      double* id_A = mxGetPr(prhs[3]);
      double* id_B = mxGetPr(prhs[4]);
      double* transform_A = mxGetPr(prhs[5]);
      double* transform_B = mxGetPr(prhs[6]);
      double* limits = mxGetPr(prhs[7]);
      index = Sim_instance->Hinge_two_transform(*id_A, *id_B,
                                        transform_A, transform_B, limits);
    }

    if (!strcmp("Hinge_one_pivot", constraint)) {
      double* id_A = mxGetPr(prhs[3]);
      double* pivot_in_A = mxGetPr(prhs[4]);
      double* axis_in_A = mxGetPr(prhs[5]);
      double* limits = mxGetPr(prhs[6]);
      index = Sim_instance->Hinge_one_pivot(*id_A, pivot_in_A,
                                    axis_in_A, limits);
    }

    if (!strcmp("Hinge_two_pivot", constraint)) {
      double* id_A = mxGetPr(prhs[3]);
      double* id_B = mxGetPr(prhs[4]);
      double* pivot_in_A = mxGetPr(prhs[5]);
      double* pivot_in_B = mxGetPr(prhs[6]);
      double* axis_in_A = mxGetPr(prhs[7]);
      double* axis_in_B = mxGetPr(prhs[8]);
      double* limits = mxGetPr(prhs[9]);
      index = Sim_instance->Hinge_two_pivot(*id_A, *id_B,
                                    pivot_in_A, pivot_in_B,
                                    axis_in_A, axis_in_B, limits);
    }

    ///////
    //Hinge2 (for cars)
    ///////
    if (!strcmp("Hinge2", constraint)){
      double* id_A = mxGetPr(prhs[3]);
      double* id_B = mxGetPr(prhs[4]);
      double* Anchor = mxGetPr(prhs[5]);
      double* Axis_1 = mxGetPr(prhs[6]);
      double* Axis_2 = mxGetPr(prhs[7]);
      double* damping = mxGetPr(prhs[8]);
      double* stiffness = mxGetPr(prhs[9]);
      double* steering_angle = mxGetPr(prhs[10]);
      index = Sim_instance->Hinge2(*id_A, *id_B, Anchor, Axis_1, Axis_2, *damping,
                               *stiffness, *steering_angle);
    }

    ///////
    //Six DOF
    ///////
    if (!strcmp("SixDOF_one", constraint)){
      double* id = mxGetPr(prhs[3]);
      double* transform_A = mxGetPr(prhs[4]);
      double* limits = mxGetPr(prhs[5]);
      index = Sim_instance->SixDOF_one(*id, transform_A, limits);
    }

    if (!strcmp("SixDOF_two", constraint)){
      double* id_A = mxGetPr(prhs[3]);
      double* id_B = mxGetPr(prhs[4]);
      double* transform_A = mxGetPr(prhs[5]);
      double* transform_B = mxGetPr(prhs[6]);
      double* limits = mxGetPr(prhs[7]);
      index = Sim_instance->SixDOF_two(*id_A, *id_B,
                               transform_A, transform_B, limits);
    }
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* ConstIndex = mxGetPr(plhs[0]);
    *ConstIndex = (double)index;
    return;
  }


  /*********************************************************************
   *
   *GETTERS FOR OBJECT POSES
   *
   **********************************************************************/


  if (!strcmp("GetTransform", cmd)) {
    //Get the type of the constraint
    char type[64];
    mxGetString(prhs[2], type, sizeof(type));
    double* id = mxGetPr(prhs[3]);
    if(!strcmp(type, "Shape")){
      double* pose = Sim_instance->GetShapeTransform(*id);
      plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
      plhs[1] = mxCreateDoubleMatrix(3, 3, mxREAL);
      double* position = mxGetPr(plhs[0]);
      double* rotation = mxGetPr(plhs[1]);
      //Position
      position[0] = pose[0];
      position[1] = pose[1];
      position[2] = pose[2];
      //Rotation
      rotation[0] = pose[3];
      rotation[1] = pose[4];
      rotation[2] = pose[5];
      rotation[3] = pose[6];
      rotation[4] = pose[7];
      rotation[5] = pose[8];
      rotation[6] = pose[9];
      rotation[7] = pose[10];
      rotation[8] = pose[11];
    }
    else if(!strcmp(type, "Constraint")){
      plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
      double* pose = Sim_instance->GetConstraintTransform(*id);
      double* position = mxGetPr(plhs[0]);
      //Position
      position[0] = pose[0];
      position[1] = pose[1];
      position[2] = pose[2];
    }
    else if(!strcmp(type, "RaycastVehicle")){
      plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
      plhs[2] = mxCreateDoubleMatrix(1, 3, mxREAL);
      plhs[4] = mxCreateDoubleMatrix(1, 3, mxREAL);
      plhs[6] = mxCreateDoubleMatrix(1, 3, mxREAL);
      plhs[8] = mxCreateDoubleMatrix(1, 3, mxREAL);
      plhs[1] = mxCreateDoubleMatrix(3, 3, mxREAL);
      plhs[3] = mxCreateDoubleMatrix(3, 3, mxREAL);
      plhs[5] = mxCreateDoubleMatrix(3, 3, mxREAL);
      plhs[7] = mxCreateDoubleMatrix(3, 3, mxREAL);
      plhs[9] = mxCreateDoubleMatrix(3, 3, mxREAL);
      double* pose = Sim_instance->GetVehicleTransform(*id);
      double* body_pos = mxGetPr(plhs[0]);
      double* wheel_fl_pos = mxGetPr(plhs[2]);
      double* wheel_fr_pos = mxGetPr(plhs[4]);
      double* wheel_bl_pos = mxGetPr(plhs[6]);
      double* wheel_br_pos = mxGetPr(plhs[8]);
      double* body_rot = mxGetPr(plhs[1]);
      double* wheel_fl_rot = mxGetPr(plhs[3]);
      double* wheel_fr_rot = mxGetPr(plhs[5]);
      double* wheel_bl_rot = mxGetPr(plhs[7]);
      double* wheel_br_rot = mxGetPr(plhs[9]);
      body_pos[0] = pose[0*12+0];
      body_pos[1] = pose[0*12+1];
      body_pos[2] = pose[0*12+2];
      body_rot[0] = pose[0*12+3];
      body_rot[1] = pose[0*12+4];
      body_rot[2] = pose[0*12+5];
      body_rot[3] = pose[0*12+6];
      body_rot[4] = pose[0*12+7];
      body_rot[5] = pose[0*12+8];
      body_rot[6] = pose[0*12+9];
      body_rot[7] = pose[0*12+10];
      body_rot[8] = pose[0*12+11];
      wheel_fl_pos[0] = pose[1*12+0];
      wheel_fl_pos[1] = pose[1*12+1];
      wheel_fl_pos[2] = pose[1*12+2];
      wheel_fl_rot[0] = pose[1*12+3];
      wheel_fl_rot[1] = pose[1*12+4];
      wheel_fl_rot[2] = pose[1*12+5];
      wheel_fl_rot[3] = pose[1*12+6];
      wheel_fl_rot[4] = pose[1*12+7];
      wheel_fl_rot[5] = pose[1*12+8];
      wheel_fl_rot[6] = pose[1*12+9];
      wheel_fl_rot[7] = pose[1*12+10];
      wheel_fl_rot[8] = pose[1*12+11];
      wheel_fr_pos[0] = pose[2*12+0];
      wheel_fr_pos[1] = pose[2*12+1];
      wheel_fr_pos[2] = pose[2*12+2];
      wheel_fr_rot[0] = pose[2*12+3];
      wheel_fr_rot[1] = pose[2*12+4];
      wheel_fr_rot[2] = pose[2*12+5];
      wheel_fr_rot[3] = pose[2*12+6];
      wheel_fr_rot[4] = pose[2*12+7];
      wheel_fr_rot[5] = pose[2*12+8];
      wheel_fr_rot[6] = pose[2*12+9];
      wheel_fr_rot[7] = pose[2*12+10];
      wheel_fr_rot[8] = pose[2*12+11];
      wheel_bl_pos[0] = pose[3*12+0];
      wheel_bl_pos[1] = pose[3*12+1];
      wheel_bl_pos[2] = pose[3*12+2];
      wheel_bl_rot[0] = pose[3*12+3];
      wheel_bl_rot[1] = pose[3*12+4];
      wheel_bl_rot[2] = pose[3*12+5];
      wheel_bl_rot[3] = pose[3*12+6];
      wheel_bl_rot[4] = pose[3*12+7];
      wheel_bl_rot[5] = pose[3*12+8];
      wheel_bl_rot[6] = pose[3*12+9];
      wheel_bl_rot[7] = pose[3*12+10];
      wheel_bl_rot[8] = pose[3*12+11];
      wheel_br_pos[0] = pose[4*12+0];
      wheel_br_pos[1] = pose[4*12+1];
      wheel_br_pos[2] = pose[4*12+2];
      wheel_br_rot[0] = pose[4*12+3];
      wheel_br_rot[1] = pose[4*12+4];
      wheel_br_rot[2] = pose[4*12+5];
      wheel_br_rot[3] = pose[4*12+6];
      wheel_br_rot[4] = pose[4*12+7];
      wheel_br_rot[5] = pose[4*12+8];
      wheel_br_rot[6] = pose[4*12+9];
      wheel_br_rot[7] = pose[4*12+10];
      wheel_br_rot[8] = pose[4*12+11];
    }

    return;
  }

  /**************************/

  // Got here, so command not recognized
  mexErrMsgTxt("Command not recognized.");
}
