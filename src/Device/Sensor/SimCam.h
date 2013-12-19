#ifndef SIMCAM_H
#define SIMCAM_H

#include <Device/SimDevices.h>
#include <ModelGraph/PhyModelGraphAgent.h>
#include <calibu/cam/CameraXml.h>
#include <CVars/CVar.h>

// a warpper for SimCam in SceneGraph for robot proxy. See more function of origional version of SimCam in SceneGraph
class SimCam
{
    public:
       // Global Variables
       unsigned int                   g_nImgWidth;
       unsigned int                   g_nImgHeight;
       unsigned int                   g_nChannel;
       SceneGraph::GLSimCam           m_Camera;     // reference camera we use
       int                            m_iCamType;
       int                            m_iFPS;
       string                         m_sDeviceName; // mesh of parent that camera attach to
       PhyModelGraphAgent             m_rPhysMGAgent;
       vector<string>                 m_vCameraModel;
       calibu::CameraRig              m_CameraRig;

       // ------------------------------------------------------------------------------------------------------------------
       bool init(Eigen::Vector6d     vInitPose,
                 string              sDeviceName,
                 int                 CameraType,
                 int                 FPS,
                 string              sCameraModel,
                 GLSceneGraph&       glGraph,
                 PhyModelGraphAgent& mPhyMGAgent
                 )
       {

           m_sDeviceName = sDeviceName;
           m_rPhysMGAgent = mPhyMGAgent;
           m_iFPS = FPS;

           cout<<"camera model file name is "<<sCameraModel<<". Device name is "<<m_sDeviceName<<endl;
           m_CameraRig = calibu::ReadXmlRig(sCameraModel);

           calibu::CameraModel theCam = m_CameraRig.cameras[0].camera;

           // // get some camera parameters
           Eigen::Matrix3d K = theCam.K();
           g_nImgWidth  = theCam.Width();
           g_nImgHeight = theCam.Height();

           string sCvar = sDeviceName+".Pose";
           const char* cCvar = sCvar.c_str();
           Eigen::Vector6d  g_vCamPose = vInitPose;
//           Eigen::Vector6d  g_vCamPose = CVarUtils::CreateCVar(
//               cCvar, Eigen::Vector6d( vInitPose),
//               "Camera's pose. Left is dominant camera." );

           // initialize cameras
           m_iCamType = CameraType;
           m_Camera.Init(&glGraph,
                         Sophus::SE3d::exp( g_vCamPose ).matrix(),
                         K, g_nImgWidth, g_nImgHeight,
                         m_iCamType );

           cout<<"[SimCam] init sim cam success. Type is "<<CameraType<<". Width is:"<<g_nImgWidth <<", "<<" Height is: "<<g_nImgHeight<<endl;
           return true;
       }

       // ------------------------------------------------------------------------------------------------------------------
       void SaveCamChannel(unsigned int CamType)
       {
           // if it is a RGBD camera
           if(CamType==5)
           {
               g_nChannel = 2;
           }
           else
           {
               g_nChannel =1;
           }
       }

       // ------------------------------------------------------------------------------------------------------------------
       // capture RGB or gray image
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


       // ------------------------------------------------------------------------------------------------------------------
       // capture depth image
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

       // ------------------------------------------------------------------------------------------------------------------
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


       // ------------------------------------------------------------------------------------------------------------------
       // get current camera pose from bullet
       Eigen::Vector6d GetCameraPoseByBody()
       {
          return m_rPhysMGAgent.m_Agent.GetEntity6Pose( m_sDeviceName );
       }

       void Update()
       {
           m_Camera.SetPoseRobot( _Cart2T(GetCameraPoseByBody()) );
//           m_Camera.SetPoseRobot( Eigen::Matrix4d::Identity() );
           m_Camera.RenderToTexture();
           m_Camera.DrawCamera();

           // simluate frame rate. This is not a clever method because the whole will sleep because of this line.
           // However, what we want is just to capture 30 image in a second. Need to improve in the futrue.
           usleep(1E0/m_iFPS);
       }

       // ------------------------------------------------------------------------------------------------------------------
       Eigen::Matrix4d GetCameraPose()
       {
          Eigen::Matrix4d dPose = m_Camera.GetPoseRobot();

//          cout<<"Get camera pose "<<endl<< _T2Cart(dPose)<<endl;
          return dPose;
       }

       void UpdateByPose(Eigen::Matrix4d DesirePose)
       {
           m_Camera.SetPoseRobot(DesirePose);
           m_Camera.RenderToTexture();
           m_Camera.DrawCamera();

           // simluate frame rate. This is not a clever method because the whole will sleep because of this line.
           // However, what we want is just to capture 30 image in a second. Need to improve in the futrue.
           usleep(1E0/m_iFPS);
       }
};





#endif // SIMCAM_H
