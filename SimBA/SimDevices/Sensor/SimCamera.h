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
  string                         m_sCameraModel;
  calibu::CameraRig              m_CameraRig;

  ///////////////////////
  /// INITIALIZER
  ///////////////////////

  SimCamera(Eigen::Vector6d vInitPose, string sDeviceName,
            SceneGraph::eSimCamType CameraType, int FPS, string sCameraModel){
    m_sDeviceName = sDeviceName;
    m_iFPS = FPS;
    m_iCamType = CameraType;
    m_CameraRig = calibu::ReadXmlRig(sCameraModel);
    m_sCameraModel = sCameraModel;
    m_vPose = vInitPose;
  }

  bool init(SceneGraph::GLSceneGraph* glGraph){
    calibu::CameraModel theCam = m_CameraRig.cameras[0].camera;
    m_nImgWidth = theCam.Width();
    m_nImgHeight = theCam.Height();
    m_Camera.Init(glGraph, _Cart2T(m_vPose), theCam.K(),
                  m_nImgWidth, m_nImgHeight, m_iCamType);
    cout<<"[SimCamera] SimCamera initialization success."<<endl;
    cout<<"Type is "<<m_sDeviceType<<". ";
    cout<<"Width is:"<<m_nImgWidth <<". Height is: "<<m_nImgHeight<<endl;
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
      return m_Camera.CaptureGrey(pImgbuf);
    }
    else if(m_iCamType == SceneGraph::eSimCamRGB){
      return m_Camera.CaptureRGB(pImgbuf);
    }
    else{
      return false;
    }
  }

  bool capture(float* pImgbuf){
    if(m_iCamType == SceneGraph::eSimCamDepth){
      return m_Camera.CaptureDepth(pImgbuf);
    }
    else{
      return false;
    }
  }

  ///////
  /// Update camera images
  ///////

  void Update(){
    m_Camera.SetPoseRobot(_Cart2T(m_vPose));
    m_Camera.RenderToTexture();
    m_Camera.DrawCamera();
    // TODO: Improve this methodology
    usleep(1E0/m_iFPS);
  }

  // Must set this through the ModelNode set.
  void UpdateByPose(Eigen::Vector6d DesiredPose){
    m_vPose = DesiredPose;
    m_Camera.SetPoseRobot(_Cart2T(DesiredPose));
    m_Camera.RenderToTexture();
    m_Camera.DrawCamera();
    usleep(1E0/m_iFPS);
  }

};

#endif // SIMCAMERA_H
