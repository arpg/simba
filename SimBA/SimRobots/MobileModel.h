#ifndef MOBILEMODEL_H
#define MOBILEMODEL_H

#include <src/Robots/PIDcontrol.h>


// the following class provide two models for the robot. N.B. We only consider x,y,th because the robot cannot fly in z axis.
// the first is one is bicycleModel (when change the angel of wheel, the robot didn't move (x,y,th didn't change) ),
// the second one is Uni-Cycle model (The robot can turn (change th) while it still in the same x,y position)
class MobileModel
{
public:

    // read command, set it in the robot. Just set the angle of front wheel. Keep x,y,th of the robot not change.
    bool BicycleModel(SimRobot &mySimRobot, float DesireVelocity, float DesireSteering)
    {
        // control the angle of the hing2joint of the wheel via a bullet interface. Keep x,y,th not change.
        // here we should have a Hinge joint position-controller that change the hing2joint of the wheel via a bullet interface.
        float fCurrentSpeed = mySimRobot.CurrentVelocity(); // get robot current speed;
        cout<<"robot current velocity is "<<fCurrentSpeed<<endl;

        PIDcontrol myPIDControl;
        myPIDControl.init(PIDcontrol::PD, DesireVelocity, fCurrentSpeed, 0, 0, 0.01, 0.01, 0.01, 0.001);

        // run pid to control velocity and steering. These to should be process in the same time
        float desire_delta_velocity;  // this is the velocity we should reach base on the requirement of PID control.
        for (int iter=0;iter!=1000;iter++)
        {
            // control velocity
                // compute desire change of velocity. Note that this is just what we desire, what we will achieve still based on the output of engine.
                desire_delta_velocity = myPIDControl.do_pid(iter);

                // updaye velocity based on the output of motor. We should apply a force on the robot and see what is its velocity after 'step time' seconds.
                myPIDControl.m_actural = myPIDControl.m_actural + mySimRobot.UpdateVelocity(desire_delta_velocity);

            // control steering
                // control steering by setting the maxium turning rate of steering. Our aim is to achieve certain angel of steerig.

            // break if satisfy error.
            if(myPIDControl.m_reach_setpoint_flag ==1){break;}
         }
             cout<<"Time for archive error rate is "<<myPIDControl.m_reach_setpoint_iter * mySimRobot.m_StepTime<<" second "<<endl;

        //
        return true;
    }


    // read command, set it in the robot. Set the th of the robot (may use PID control). keep x,y, of the robot.
    bool UniCycleModel()
    {
        // control velocity


        // control steering
        // change the angel (th) of the robot. keep x,y the same. Notice that is a turn rate for th

        //
        return true;
    }
};




#endif // MOBILEMODEL_H
