#ifndef SIMCAM_H
#define SIMCAM_H

#include <Device/SimDevices.h>
#include <ModelGraph/PhyModelGraphAgent.h>


class SimCam
{
    public:
       // Global Variables
       unsigned int                   g_nImgWidth;
       unsigned int                   g_nImgHeight;
       SceneGraph::GLSimCam           m_Camera;     // reference camera we use
       int                            m_iCamType;
       int                            m_FPS;
       string                         m_sDeviceName; // mesh of parent that camera attach to
       PhyModelGraphAgent             m_rPhysMGAgent;
       vector<string>                 m_vCameraModel;

       bool init(Eigen::Vector6d  vInitPose, string sDeviceName, int CameraType, string sCameraModel, GLSceneGraph&  glGraph, PhyModelGraphAgent&  mPhyMGAgent)
       {
           m_sDeviceName = sDeviceName;
           m_rPhysMGAgent = mPhyMGAgent;

           cout<<"[SimCam] try to use "<<sCameraModel<<endl;
           mvl::CameraModel CamModel( sCameraModel );

           // get some camera parameters
           Eigen::Matrix3d K = CamModel.K();
           g_nImgWidth  = CamModel.Width();
           g_nImgHeight = CamModel.Height();

           string sCvar = sDeviceName+".Pose";
           const char* cCvar = sCvar.c_str();
           Eigen::Vector6d  g_vCamPose = CVarUtils::CreateCVar( cCvar, Eigen::Vector6d( vInitPose), "Camera's pose. Left is dominant camera." );

           // initialize cameras
           m_iCamType = CameraType;
           m_Camera.Init(&glGraph, mvl::Cart2T( g_vCamPose ), K, g_nImgWidth, g_nImgHeight, m_iCamType );

           cout<<"[SimCam] init sim cam success. Type is "<<CameraType<<". Width is:"<<g_nImgWidth <<", "<<" Height is: "<<g_nImgHeight<<endl;
           return true;
       }


       bool capture(char* pImgbuf)
       {
           if (m_iCamType == 1)
           {
               if(m_Camera.CaptureGrey(pImgbuf)==true)
               {
                    return true;
               }
               else
               {
                    cout<<"[SimCam] capture gray fail"<<endl;
                    return false;
               }
           }
           else if (m_iCamType == 2)
           {
               if(m_Camera.CaptureRGB(pImgbuf)==true)
               {
                   return true;
               }
               else
               {
                   cout<<"[SimCam] capture rgb fail"<<endl;
                   return false;
               }
           }
           else
           {
              return false;
           }
       }


       bool capture(float* pImgbuf)
       {
           if(m_Camera.CaptureDepth(pImgbuf)==true)
           {
//              NormalizeDepth(pImgbuf,sizeof(pImgbuf)); // normalize the depth value to 0-1
                return true;
           }
           else
           {
               cout<<"capture depth image fail"<<endl;
               return false;
           }
       }

       // this function NormalizeDepth to zero and one
       inline void NormalizeDepth( float* Depth, unsigned int Size)
       {
           // find max depth
           float MaxDepth = 0;

           for( unsigned int ii = 0; ii < Size; ii++ ) {
               if( MaxDepth < Depth[ii] ) {
                   MaxDepth = Depth[ii];
               }
           }

           if( MaxDepth == 0 ) {
               return;
           }

           // normalize
           for( unsigned int ii = 0; ii < Size; ii++ ) {
               Depth[ii] = Depth[ii] / MaxDepth;
           }
       }

       // update is the same for all types of sim cam
       void Update()
       {
           m_Camera.SetPoseRobot(mvl::Cart2T(GetCameraPose()));
           m_Camera.RenderToTexture();
           m_Camera.DrawCamera();
       }


       Eigen::Vector6d GetCameraPose()
       {
          Eigen::Vector6d CameraPose;
          m_rPhysMGAgent.m_Agent.GetEntity6Pose(m_sDeviceName,CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5]);

//              cout<<"[SimCam] New pose for sim cam is "<<CameraPose[0]<<", "<<CameraPose[1]<<", "<<CameraPose[2]<<", "
//                  <<", "<<CameraPose[3]<<", "<<CameraPose[4]<<", "<<CameraPose[5]<<endl;
          return CameraPose;
       }


       void UpKey()
       {
           Eigen::Vector6d CameraPose;
           m_rPhysMGAgent.m_Agent.GetEntity6Pose(m_sDeviceName,CameraPose[0],CameraPose[1],CameraPose[2],CameraPose[3],CameraPose[4],CameraPose[5]);
       }
};





#endif // SIMCAM_H
