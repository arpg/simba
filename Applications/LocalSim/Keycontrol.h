#ifndef KEYCONTROL_H
#define KEYCONTROL_H
#include <stdio.h>
#include <string.h>
#include <ModelGraph/PhysicsEngine.h>
#include <GL/gl.h>
#include <fstream>
#include <iostream>

extern int pose_index;
extern double rot_degree;
extern int which_key[2];
extern bool Key_Status[4];
extern bool Press_Status[2];
extern float Ini_speed;
extern bool Is_firtime;
extern bool save_key[2];
extern bool speed_state[4];
extern bool side[4];


 void Btdown_Callback(unsigned char key, int x, int y);


 void Btup_Callback(unsigned char key, int x, int y);

 void Vehicle_Forward(VehiclePtr pVeh);



 void Vehicle_Backward(VehiclePtr pVeh);



 VehiclePtr Vehicle_Move(VehiclePtr pVeh);

 VehiclePtr Vehicle_Move2(VehiclePtr pVeh);

std::vector <long double> Cam_Load_Path (std::string path_file);
//void Cam_Move_Path (VehiclePtr pVeh,std::vector<float> path_xy,int pace);


VehiclePtr Vehicle_auto_move(VehiclePtr pVeh);

void DrawPoints(double x, double y, double z);

#endif // KEYCONTROL_H
