#ifndef SIMDEVICEMANAGER_H
#define SIMDEVICEMANAGER_H

#include <Device/SimDeviceInfo.h>
#include <Device/SimDevices.h>
#include <ModelGraph/PhyModelGraphAgent.h>
#include <URDFParser/RobotProxyURDFParser.h>

using namespace std;


class SimDeviceManager
{

public:
    PhyModelGraphAgent              m_rPhyMGAgent;
    vector<SimDeviceInfo>           m_SimDevices;

    //----------------------------------------------------//<DeviceName, SimGPS*>
    map<string, SimGPS*>               m_SimGPSList;
    map<string, SimVicon*>             m_SimViconList;
    map<string, SimCam*>               m_SimCamList;
    map<string, SimLaser2D*>           m_SimLaser2DList;
    map<string, SimLaser3D*>           m_SimLaser3DList;
    map<string, SimpleController*>     m_SimpleControllerList;
    map<string, CarController*>        m_CarControllerList;


    SimDeviceManager()
    {

    }


    // ------------------------------------------------------------------------------------------------
    bool Init(PhyModelGraphAgent& pPhyMGAgent, GLSceneGraph&  rSceneGraph, tinyxml2::XMLDocument& doc, string sProxyName)
    {
        ParseDevice(doc, m_SimDevices, sProxyName);
        m_rPhyMGAgent = pPhyMGAgent;
        InitDevices(rSceneGraph);
        return true;
    }


    // ------------------------------------------------------------------------------------------------
    void InitDevices(SceneGraph::GLSceneGraph&  rSceneGraph)
    {
        for(unsigned int i = 0; i!= m_SimDevices.size(); i++)
        {
            SimDeviceInfo Device = m_SimDevices[i];
            string sDeviceType = Device.sDeviceType;

            if(sDeviceType == "Camera")
            {
                 InitCamDevice(Device,rSceneGraph);
            }

            if(sDeviceType == "GPS")
            {

            }

            if(sDeviceType == "Vicon")
            {
                InitViconDevice(Device);
            }

            cout<<"get device type "<<sDeviceType<<endl;
            if(sDeviceType == "Controller")
            {
                InitController(Device);
            }
        }
        cout<<"[SimDeviceManager] init all devices success. "<<endl;
    }


    // ------------------------------------------------------------------------------------------------
    void InitCamDevice(SimDeviceInfo& Device, SceneGraph::GLSceneGraph&  rSceneGraph)
    {
        string sCameraName = Device.sDeviceName;
        int iFPS = Device.m_iFPS;
        for(unsigned int j = 0; j!= Device.m_vSensorList.size(); j++)
        {
            string sSensorName = Device.m_vSensorList[j]; // e.g. LCameraRGB. This may be bad

            Eigen::Vector6d initPose;
            m_rPhyMGAgent.m_Agent.GetEntity6Pose(sSensorName,initPose[0],initPose[1],initPose[2],initPose[3],initPose[4],initPose[5]);

//            vector<string>::iterator it = Device.m_vModel.begin();
            string sCameraModel="/Users/faradazerage/code/simba/src/Device/lcmod.xml"; // *it;
//            Device.m_vModel.erase(it);

//            cout<<"[SimCam] The Camera model use is: "<<sCameraModel<<endl;

            SimCam* pSimCam = new SimCam();

            if(sSensorName == "Gray" + sCameraName)           //---------- init gray Cam
            {
                    cout<<"[SimDeviceManager] try to init Gray camera, name is "<<sCameraName<<endl;
                    pSimCam->init(initPose, sSensorName, eSimCamLuminance, iFPS, sCameraModel, rSceneGraph, m_rPhyMGAgent );
            }
            else if(sSensorName == "RGB" + sCameraName)       //---------- init RGB Cam
            {
                    cout<<"[SimDeviceManager] try to init RGB camera, name is "<<sSensorName<<endl;
                    pSimCam->init(initPose, sSensorName, eSimCamRGB,  iFPS, sCameraModel, rSceneGraph, m_rPhyMGAgent );
                    std::cout<<"There was an error here: "<<sCameraModel<<std::endl;

            }
            else if(sSensorName == "Depth" + sCameraName)     //---------- init Depth Cam
            {
                    cout<<"[SimDeviceManager] try to init Depth camera, name is "<<sSensorName<<endl;
                    pSimCam->init(initPose,sSensorName,eSimCamLuminance|eSimCamDepth, iFPS, sCameraModel, rSceneGraph, m_rPhyMGAgent );
            }

            m_SimCamList.insert(pair<string,SimCam*>(sSensorName,pSimCam));
        }
    }


    // ------------------------------------------------------------------------------------------------
    void InitViconDevice(SimDeviceInfo& Device)
    {
          string sDeviceName = Device.sDeviceName;
          string sBodyName = Device.sBodyName;
          SimVicon* pSimVicon = new SimVicon;
          pSimVicon->init(sDeviceName, sBodyName, m_rPhyMGAgent );
          m_SimViconList.insert(pair<string, SimVicon*>(sDeviceName,pSimVicon));
    }


    void InitController(SimDeviceInfo& Device)
    {
         string sDeviceName = Device.sDeviceName;
         string sDeviceMode = Device.sDeviceMode;
         string sRobotName = Device.sRobotName;

         if(sDeviceMode == "SimpleController")
         {
             SimpleController* pSimpleController = new SimpleController;
             pSimpleController->init(sDeviceName, sRobotName, sDeviceName, m_rPhyMGAgent );
             m_SimpleControllerList.insert(pair<string, SimpleController*>(sDeviceName, pSimpleController));
             cout<<"add "<<sDeviceName<<" success "<<endl;
         }

         if(sDeviceMode == "CarController")
         {
//             CarController* pCarController = new CarController;
         }

    }



    // ------------------------------------------------------------------------------------------------
    // update all device currently we use
    void UpdateAlLDevice()
    {
        if(m_SimCamList.size()!=0)
        {
            map<string, SimCam*>::iterator iter;
            for(iter = m_SimCamList.begin();iter!= m_SimCamList.end();iter++)
            {
                iter->second->Update();
            }
        }

        if(m_SimGPSList.size()!=0)
        {
            map<string, SimGPS*>::iterator iter;
            for(iter = m_SimGPSList.begin();iter!= m_SimGPSList.end();iter++)
            {
                iter->second->Update();
            }
        }
        if(m_SimViconList.size()!=0)
        {
            map<string, SimVicon*>::iterator iter;
            for(iter = m_SimViconList.begin();iter!= m_SimViconList.end();iter++)
            {
                iter->second->Update();
            }
        }

    }



    // ------------------------------------------------------------------------------------------------
    // get pointer of device
    SimpleController* GetSimpleController(string name)
    {
        std::map<string, SimpleController*>::iterator iter = m_SimpleControllerList.find(name);
        if(iter == m_SimpleControllerList.end())
        {
            cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
        }
        SimpleController* pSimpleController = iter->second;
        return pSimpleController;
    }

    SimCam* GetSimCam(string name)
    {
       std::map<string, SimCam*>::iterator iter = m_SimCamList.find(name);
       if(iter == m_SimCamList.end())
       {
           cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
       }
       SimCam* pSimCam = iter->second;
       return pSimCam;
    }

    SimGPS* GetSimGPS(string name)
    {
       std::map<string, SimGPS*>::iterator iter =m_SimGPSList.find(name);
       if(iter == m_SimGPSList.end())
       {
           cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
       }
       SimGPS* pSimGPS = iter->second;
       return pSimGPS;
    }

    SimVicon* GetSimVecon(string name)
    {
        std::map<string, SimVicon*>::iterator iter = m_SimViconList.find(name);
        if(iter == m_SimViconList.end())
        {
            cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
        }
        SimVicon* pSimVicon = iter->second;
        return pSimVicon;
    }
};


#endif // SIMDEVICEMANAGER_H
