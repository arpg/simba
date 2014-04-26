#include "Keycontrol.h"

int pose_index=0;
 double rot_degree=0;
 int which_key[2]={0,0};
 bool Key_Status[4]={false,false,false,false};
 bool Press_Status[2]={false,false};
 float Ini_speed;
 bool Is_firtime=true;
 bool save_key[2]={false,false};
 bool speed_state[4]={false,false,false,false};
 bool side[4]={true,false,false,false};
 btVector3 pos_recod[4];
 btVector3 pos;
 int one_round=0;
 bool finish[2]={false,false};
 Eigen::Vector6d Pose[100];
Eigen::Vector3d ALL_POS[100];

void Btdown_Callback(unsigned char key, int x, int y)
{

    switch(key)    //dectecting key down event
    {
    case 'w':
        Key_Status[0]=true;
        which_key[0]=1;
        which_key[1]=0;

       break;
    case 's':
        Key_Status[1]=true;
        which_key[0]=0;
        which_key[1]=1;
        break;
    case 'a':
        Key_Status[2]=true;
        break;
    case 'd':
        Key_Status[3]=true;

        break;
    case 'c':
        save_key[0]=true;
       break;
    case 'f':
        finish[0]=true;
    break;
    }
}

void Btup_Callback(unsigned char key, int x, int y)
{
    switch(key)   //Detecting key up events
    {
    case 'w':
        Key_Status[0]=false;
       break;
    case 's':
        Key_Status[1]=false;
        break;
    case 'a':
        Key_Status[2]=false;
       break;
    case 'd':
        Key_Status[3]=false;
        break;
    case 'c':
        save_key[1]=true;
       break;
    case 'f':
        finish[1]=true;
        break;
    }
}

void Vehicle_Forward(VehiclePtr pVeh)
{
    switch(Key_Status[0])
    {
    case true:
        if(Press_Status[0]==false)
        {
            pVeh->setBrake(0,2);
            pVeh->setBrake(0,3);
            pVeh->setBrake(0,0);
            pVeh->setBrake(0,1);

            if(abs(pVeh->getCurrentSpeedKmHour())<10)
            {
              pVeh->applyEngineForce(10, 2);
              pVeh->applyEngineForce(10, 3);
            }
            if(abs(pVeh->getCurrentSpeedKmHour())>=10)
            {
                pVeh->applyEngineForce(0, 2);
                pVeh->applyEngineForce(0, 3);
                Press_Status[0]=true;
            }

        }

        break;
    case false:
        if(abs(pVeh->getCurrentSpeedKmHour())>=5)
        {
            int f=10*abs(pVeh->getCurrentSpeedKmHour());
          pVeh->applyEngineForce(-f, 2);
          pVeh->applyEngineForce(-f, 3);

        }
        else
        {
            pVeh->applyEngineForce(0, 2);
            pVeh->applyEngineForce(0, 3);

           pVeh->setBrake(5,0);
           pVeh->setBrake(5,1);
           pVeh->setBrake(5,2);
           pVeh->setBrake(5,3);
        }

          Press_Status[0]=false;
        break;
    }

}


void Vehicle_Backward(VehiclePtr pVeh)
{
    switch(Key_Status[1])
    {
    case true:
        if(Press_Status[1]==false)
        {
            pVeh->setBrake(0,0);
            pVeh->setBrake(0,1);
            pVeh->setBrake(0,2);
            pVeh->setBrake(0,3);

            if(abs(pVeh->getCurrentSpeedKmHour())<5)
            {
              pVeh->applyEngineForce(-10, 2);
              pVeh->applyEngineForce(-10, 3);
            }
            if(abs(pVeh->getCurrentSpeedKmHour())>=5)
            {
                pVeh->applyEngineForce(0, 2);
                pVeh->applyEngineForce(0, 3);
                Press_Status[1]=true;
            }
           // Press_Status[1]=true;
        }

        break;
    case false:

        if(abs(pVeh->getCurrentSpeedKmHour())>=5)
        {
            int f=10*abs(pVeh->getCurrentSpeedKmHour());
            pVeh->applyEngineForce(f, 0);
            pVeh->applyEngineForce(f, 1);

        }
        else
        {
            pVeh->applyEngineForce(0, 2);
            pVeh->applyEngineForce(0, 3);

          pVeh->setBrake(5,2);
          pVeh->setBrake(5,3);
          pVeh->setBrake(5,0);
          pVeh->setBrake(5,1);
         }

          Press_Status[1]=false;
        break;
    }

}

void Vehicle_Turning(VehiclePtr pVeh)
{
    switch(Key_Status[2])
    {
    case true:
        if(rot_degree<=M_PI/4)
        {
          rot_degree=rot_degree+M_PI/100;
          pVeh->setSteeringValue(rot_degree, 0);
          pVeh->setSteeringValue(rot_degree, 1);
        }

        break;
    case false:
        break;

    }
    switch(Key_Status[3])
    {
    case true:
        if(rot_degree>=-M_PI/4)
        {
            rot_degree=rot_degree-M_PI/100;
            pVeh->setSteeringValue(rot_degree, 0);
            pVeh->setSteeringValue(rot_degree, 1);
        }

        break;
    case false:
        break;

        }
}



VehiclePtr Vehicle_Move(VehiclePtr pVeh)
{

    glutKeyboardFunc(&Btdown_Callback);
    glutKeyboardUpFunc(&Btup_Callback);
    if(which_key[0]==1 && which_key[1]==0)
    {
        Vehicle_Forward(pVeh);
        Vehicle_Turning(pVeh);
        pVeh->resetSuspension ();
        return pVeh;
    }

    if(which_key[0]==0 && which_key[1]==1)
    {
        Vehicle_Backward(pVeh);
        Vehicle_Turning(pVeh);
        pVeh->resetSuspension ();
        return pVeh;
    }

    return pVeh;

}

VehiclePtr Vehicle_Move2(VehiclePtr pVeh)
{
    glutKeyboardFunc(&Btdown_Callback);
    glutKeyboardUpFunc(&Btup_Callback);
    btRigidBody* Vehiclerb;
    Vehiclerb=pVeh->getRigidBody();
    btVector3 speed;
    if(Key_Status[0]==true)
    {
     speed.setY(500);
     speed.setX(0);
     speed.setZ(0);
     Vehiclerb->setLinearVelocity(speed);
    }

    if(Key_Status[1]==true)
    {
     speed.setY(-500);
     speed.setX(0);
     speed.setZ(0);
     Vehiclerb->setLinearVelocity(speed);
    }

    if(Key_Status[2]==true)
    {
     speed.setY(0);
     speed.setX(500);
     speed.setZ(0);
     Vehiclerb->setLinearVelocity(speed);
    }

    if(Key_Status[3]==true)
    {
     speed.setY(0);
     speed.setX(-500);
     speed.setZ(0);
     Vehiclerb->setLinearVelocity(speed);
    }
    if(Key_Status[3]==false && Key_Status[2]==false && Key_Status[1]==false && Key_Status[0]==false)
    {
     speed.setY(0);
     speed.setX(0);
     speed.setZ(0);
     Vehiclerb->setLinearVelocity(speed);
    }
    if(save_key[0]==true && save_key[1]==true)  //put all the position in a vector
    {
        cout<<"x="<<Vehiclerb->getCenterOfMassPosition().getX()<<endl;
        cout<<"y="<<Vehiclerb->getCenterOfMassPosition().getY()<<endl;
        cout<<"z="<<Vehiclerb->getCenterOfMassPosition().getZ()<<endl;
        ALL_POS[pose_index]<<Vehiclerb->getCenterOfMassPosition().getX(),
                Vehiclerb->getCenterOfMassPosition().getY(),
                Vehiclerb->getCenterOfMassPosition().getZ();
        pose_index=pose_index+1;
        save_key[0]=false;
        save_key[1]=false;
    }
    if(finish[0]==true && finish[1]==true) //save all the pos in a file
    {
        std::ofstream f1;
        f1.open("ALL_POS.txt");
        for(int i=0;i<=pose_index-1;i++)
        {
          f1<<ALL_POS[i]<<'\n'<<endl;
        }
        f1.close();
        cout<<"saved!"<<endl;
        finish[0]=false;
        finish[1]=false;
    }
    return pVeh;
}



VehiclePtr Vehicle_auto_move(VehiclePtr pVeh)
{

    btVector3 speed;
    btRigidBody* Vehiclerb;
    Vehiclerb=pVeh->getRigidBody();
    btQuaternion rot1;
    btTransform rot2;
    if(int(round(pos.getY()))<=-2600 && int(round(pos.getY()))>=-2800 && pos.getX()==pos_recod[0].getX() && side[0]==true)   //turning corner 1
    {

          speed.setX(0);
          speed.setY(0);
          speed.setZ(0);
          Vehiclerb->setLinearVelocity(speed);
          //rot1.setEuler(0,0,0);
          //rot2.setOrigin(pos);
         // rot2.setRotation(rot1);
         // Vehiclerb->proceedToTransform(rot2);
          rot1.setEuler(0,0,-M_PI/2);
          rot2.setOrigin(pos);
          rot2.setRotation(rot1);
          Vehiclerb->proceedToTransform(rot2);
          side[0]=false;
          side[1]=true;
          side[2]=false;
          side[3]=false;
          one_round++;

    }


    if(int(round(pos.getX()))<=-2300 && int(round(pos.getX()))>=-2500 && int(round(pos.getY()))==int(round(pos_recod[1].getY())) && side[1]==true)   //turning corner 2
    {

          speed.setX(0);
          speed.setY(0);
          speed.setZ(0);
          Vehiclerb->setLinearVelocity(speed);
         // rot1.setEuler(0,0,0);
         // rot2.setOrigin(pos);
         // rot2.setRotation(rot1);
         // Vehiclerb->proceedToTransform(rot2);
          rot1.setEuler(0,0,-M_PI);
          rot2.setOrigin(pos);
          rot2.setRotation(rot1);
          Vehiclerb->proceedToTransform(rot2);
          side[0]=false;
          side[1]=false;
          side[2]=true;
          side[3]=false;
          one_round++;

    }



    if(int(round(pos.getY()))<=0 && int(round(pos.getY()))>=-20 && int(round(pos.getX()))==int(round(pos_recod[2].getX())) && side[2]==true)   //turning corner 3
    {

          speed.setX(0);
          speed.setY(0);
          speed.setZ(0);
          Vehiclerb->setLinearVelocity(speed);
         // rot1.setEuler(0,0,0);
        //  rot2.setOrigin(pos);
        //  rot2.setRotation(rot1);
        //  Vehiclerb->proceedToTransform(rot2);
          rot1.setEuler(0,0,-3*M_PI/2);
          rot2.setOrigin(pos);
          rot2.setRotation(rot1);
          Vehiclerb->proceedToTransform(rot2);
          side[0]=false;
          side[1]=false;
          side[2]=false;
          side[3]=true;
          one_round++;

    }

    if(int(round(pos.getX()))<=0 && int(round(pos.getX()))>=-20 && int(round(pos.getY()))==int(round(pos_recod[3].getY())) && side[3]==true)   //turning corner 4
    {

          speed.setX(0);
          speed.setY(0);
          speed.setZ(0);
          Vehiclerb->setLinearVelocity(speed);
         // rot1.setEuler(0,0,0);
         // rot2.setOrigin(pos);
         // rot2.setRotation(rot1);
         // Vehiclerb->proceedToTransform(rot2);
          rot1.setEuler(0,0,-2*M_PI);
          rot2.setOrigin(pos);
          rot2.setRotation(rot1);
          Vehiclerb->proceedToTransform(rot2);
          side[0]=true;
          side[1]=false;
          side[2]=false;
          side[3]=false;
          one_round++;

    }

    if(one_round==4)   //the car has gone a round,save the poses
    {

        std::ofstream f1;
        f1.open("Pose.txt");
        for(int i=0;i<=99;i++)
        {

         f1<<Pose[i]<<endl;
          Pose[i].setZero(6);
        }
        f1.close();
          one_round=0;
    }


    if(side[0]==true)  //left side
    {
        if(speed_state[0]==false)
        {
          speed.setX(0);
           speed.setY(-500);
           speed.setZ(0);
           Vehiclerb->setLinearVelocity(speed);
        }
        pos_recod[0]=Vehiclerb->getCenterOfMassPosition();
        pos=Vehiclerb->getCenterOfMassPosition();
        speed_state[0]=true;
        speed_state[1]=false;
        speed_state[2]=false;
        speed_state[3]=false;

    }


    if(side[1]==true)  //upper side
    {
        if(speed_state[1]==false)
        {
          speed.setX(-500);
           speed.setY(0);
           speed.setZ(0);
           Vehiclerb->setLinearVelocity(speed);
        }
        pos_recod[1]=Vehiclerb->getCenterOfMassPosition();
        pos=Vehiclerb->getCenterOfMassPosition();
        speed_state[0]=false;
        speed_state[1]=true;
        speed_state[2]=false;
        speed_state[3]=false;

    }


    if(side[2]==true)  //right side
    {
        if(speed_state[2]==false)
        {
          speed.setX(0);
           speed.setY(500);
           speed.setZ(0);
           Vehiclerb->setLinearVelocity(speed);
        }
        pos_recod[2]=Vehiclerb->getCenterOfMassPosition();
        pos=Vehiclerb->getCenterOfMassPosition();
        speed_state[0]=false;
        speed_state[1]=false;
        speed_state[2]=true;
        speed_state[3]=false;

    }

    if(side[3]==true)  //down side
    {
        if(speed_state[3]==false)
        {
          speed.setX(500);
           speed.setY(0);
           speed.setZ(0);
           Vehiclerb->setLinearVelocity(speed);
        }
        pos_recod[3]=Vehiclerb->getCenterOfMassPosition();
        pos=Vehiclerb->getCenterOfMassPosition();
        speed_state[0]=false;
        speed_state[1]=false;
        speed_state[2]=false;
        speed_state[3]=true;
    }
    cout<<pos.getX()<<endl;
    cout<<pos.getY()<<endl;
    cout<<"z="<<pos.getZ()<<endl;
    if(pose_index<=99)
    {
      Pose[pose_index]<<pos.getX(),pos.getY(),pos.getZ(),Vehiclerb->getCenterOfMassTransform().getRotation().getX(),
            Vehiclerb->getCenterOfMassTransform().getRotation().getY(),
            Vehiclerb->getCenterOfMassTransform().getRotation().getY();
      pose_index++;
    }   
    return pVeh;
}

std::vector <long double> Cam_Load_Path (std::string path_file)
{
    std::vector <long double> path_xy;
    ifstream path(path_file,ios::binary);
         float value;

         while(1)
         {
             path>>value;
             if(path.eof())
             {
                 break;
             }
             path_xy.push_back(value);
         }


    return path_xy;

}


///////////////////////
class Keycontrol
{
public:
};



