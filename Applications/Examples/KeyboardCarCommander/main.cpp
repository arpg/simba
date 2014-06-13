#include <iostream>

#include <HAL/Car/CarDevice.h>
#include <pangolin/pangolin.h>
#include <boost/bind.hpp>

/**********************
 * KEYBOARD CAR COMMAND
 * This is a test program that uses the keyboard to inut commands to
 * a RaycastVehicle in SimBA.
 * ********************/

/// Keyboard commands
class KeyCarController{
public:

  KeyCarController(std::string parameters){
    car = new hal::Car(parameters);
    torque = 0;
    steering = 0;
  }

  void GoForward(){
    torque = torque + .2;
    if(torque>1){
      torque = 1;
    }
  }

  void GoBack(){
    torque = torque - .2;
    if(torque<-1){
      torque = -1;
    }
  }

  void GoRight(){
    steering = steering + .1;
    if(steering>1){
      steering = 1;
    }
  }

  void GoLeft(){
    steering = steering - .1;
    if(steering<-1){
      steering = -1;
    }
  }

  void ApplyCommands(){
    // We must give a command_time (in this case, a 30th of a second)
    bool applied = car->ApplyCommand(1, 20, 1.0/30.0);
    if(applied){
      std::cout<<"sent command!";
    }
  }

  hal::Car* car;
  double torque;
  double steering;

};

/****************
 * MAIN LOOP
 * **************/

int main(){
  // This should do it...
  // IMPORTANT: Make sure that you have the name of the sim that you want to
  // connect with in the properties string (e.g. sim=LocalSim). Otherwise,
  // when we have multiple sims on the same network, we won't be able to
  // identify what to control
  KeyCarController KeyCar("NodeCar:[name=VehicleController,sim=Ricky]//");

  while(1){
    KeyCar.GoForward();
    KeyCar.GoRight();
    KeyCar.ApplyCommands();
  }

}
