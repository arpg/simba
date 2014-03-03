#ifndef SIMCAMERA_H
#define SIMCAMERA_H

#include <SimDevices/SimDeviceInfo.h>
#include <SceneGraph/SimCam.h>
#include <calibu/cam/CameraXml.h>

// A wrapper for SceneGraph::SimCam for LocalSim.

/////////////////
////////////////
////////////////
// TODO: Strip away the ModelGraph references from this class.
// It should hold information; it does not initialize anything
// That functionality has now been moved to RenderEngine
/////////////////
////////////////
/////////////////

class SimCamera: public SimDeviceInfo
{
public:

  ///////////////////////
  /// GLOBAL VARIABLES
  ///////////////////////

  unsigned int                   m_nImgWidth;
  unsigned int                   m_nImgHeight;
  unsigned int                   m_nChannels;

  // Reference camera we use
  SceneGraph::GLSimCam           m_Camera;
  int                            m_iBaseline;
  SceneGraph::eSimCamType        m_iCamType;
  int                            m_iFPS;
  // Path of model.xml file for the sensor
  string                         m_sModel;

  // Physical entity that camera attaches to

  vector<string>                 m_vCameraModel;
  calibu::CameraRig              m_CameraRig;

  ///////////////////////
  /// INITIALIZER
  ///////////////////////

  SimCamera(){
  }

  bool init(Eigen::Vector6d vInitPose, string sDeviceName,
            SceneGraph::eSimCamType CameraType, int FPS, string sCameraModel){
    m_sDeviceName = sDeviceName;
    m_iFPS = FPS;
    cout<<"[SimCamera] camera model file name is "<<sCameraModel<<endl;
    cout<<"Device name is "<<m_sDeviceName<<endl;
    m_CameraRig = calibu::ReadXmlRig(sCameraModel);
    calibu::CameraModel theCam = m_CameraRig.cameras[0].camera;

    // get some camera parameters
    Eigen::Matrix3d K = theCam.K();
    m_nImgWidth       = theCam.Width();
    m_nImgHeight      = theCam.Height();

    // initialize cameras
    m_iCamType = CameraType;

    // This now happens in RenderEngine
    //    m_Camera.Init(&pModelGraph->m_Render.m_glGraph,
    //                  Sophus::SE3d::exp( vInitPose ).matrix(),
    //                  K, m_nImgWidth, m_nImgHeight, m_iCamType );

    cout<<"[SimCamera] init sim cam success. Type is "<<
          CameraType<<". Width is:"<<m_nImgWidth <<", "
       <<" Height is: "<<m_nImgHeight<<endl;
    return true;
  }

  ///////////////////////
  /// CAMERA FUNCTIONS
  ///////////////////////

  void SaveCamChannel(unsigned int CamType){
    (CamType==SceneGraph::eSimCamRGB) ? m_nChannels = 2 : m_nChannels = 1;
  }

  inline void NormalizeDepth( float* Depth, unsigned int Size){
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
    // Normalize
    for( unsigned int ii = 0; ii < Size; ii++ ) {
      Depth[ii] = Depth[ii] / MaxDepth;
    }
  }

  /////////
  // We need two capture functions:
  // RGB and Grey take char*, while Depth takes float*
  /////////

  bool capture(char* pImgbuf){
    if(m_iCamType == SceneGraph::eSimCamLuminance){
      m_Camera.CaptureGrey(pImgbuf);
      return true;
    }
    else if(m_iCamType == SceneGraph::eSimCamRGB){
      m_Camera.CaptureRGB(pImgbuf);
      return true;
    }
    else{
      return false;
    }
  }

  bool capture(float* pImgbuf){
    if(m_iCamType == SceneGraph::eSimCamDepth){
      m_Camera.CaptureDepth(pImgbuf);
    }
  }

  ///////
  /// Update camera images
  ///////

  void Update(){
    m_Camera.SetPoseRobot( _Cart2T(GetCameraPoseByBody()) );
    m_Camera.RenderToTexture();
    m_Camera.DrawCamera();

    // TODO: Improve this methodology
    // simluate frame rate. This is not a clever method because the whole app
    // will sleep because of this line. However, what we want is just to
    // capture 30 image in a second.
    usleep(1E0/m_iFPS);
  }

  void UpdateByPose(Eigen::Matrix4d DesirePose){
    m_Camera.SetPoseRobot(DesirePose);
    m_Camera.RenderToTexture();
    m_Camera.DrawCamera();
    usleep(1E0/m_iFPS);
  }

  ///////////////////////
  /// GETTERS
  ///////////////////////

  Eigen::Vector6d GetCameraPoseByBody(){
//    return m_pModelGraph->m_Phys.GetEntity6Pose( m_sDeviceName );
  }

  Eigen::Matrix4d GetCameraPose(){
    Eigen::Matrix4d dPose = m_Camera.GetPoseRobot();

    //          cout<<"Get camera pose "<<endl<< _T2Cart(dPose)<<endl;
    return dPose;
  }

};

#endif // SIMCAMERA_H
