#include "URDF_Parser.h"

////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////

URDF_Parser::URDF_Parser(){
}

////////////////////////////////////////////////////////////
/// PARSE WORLD.XML IN ROBOTPROXY
////////////////////////////////////////////////////////////

bool URDF_Parser::ParseWorld(const char* filename,
                             WorldManager& mWorldManager){
  XMLDocument doc;
  if(doc.LoadFile(filename) !=0){
    printf("Cannot open %s\n", filename);
    return false;
  }

  XMLElement *pParent=doc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();

  // read high level parent (root parent)
  while (pElement){
    const char* sRootContent = pElement->Name();
    if(strcmp(sRootContent,"base")==0){
      string sMesh(pElement->Attribute("mesh"));
      mWorldManager.m_sMesh = sMesh;
      mWorldManager.iScale =::atoi( pElement->Attribute("scale"));
      mWorldManager.iMass =::atoi( pElement->Attribute("mass"));
      mWorldManager.vWorldPose =
          GenNumFromChar(pElement->Attribute("worldpose"));
      mWorldManager.vRobotPose=
          GenNumFromChar(pElement->Attribute("robotpose"));
      mWorldManager.vLightPose=
          GenNumFromChar(pElement->Attribute("lightpose"));
    }

    pElement=pElement->NextSiblingElement();
  }

  mWorldManager.PrintAll();

  return true;
}

////////////////////////////////////////////////////////////
/// PARSE ROBOT.XML FOR ROBOT PARTS AND BUILD INTO ROBOTPROXY
/// The robot name format: robotname@proxyname. e.g. robot1@proxy1.
/// All devices will live under this name.
/// The name of any robot body is: BodyName@RobotName@ProxyName
/// The name of any robot joint is: JointName@RobotName@ProxyName
////////////////////////////////////////////////////////////

bool URDF_Parser::ParseRobot(XMLDocument* doc,
                             RobotModel& m_RobotModel,
                             string sProxyName){

  cout<<"[ParseRobot] Start Parsing Robot"<<endl;
  XMLElement *pParent=doc->RootElement();
  string sRobotName(GetAttribute(pParent, "name"));
  sRobotName = sRobotName+"@"+sProxyName;
  string sRobotType(GetAttribute(pParent, "type"));
  m_RobotModel.SetName(sRobotName);
  XMLElement *pElement = pParent->FirstChildElement();

  if(!sRobotType.compare("RaycastVehicle")){
    cout<<"[URDF_Parser] Try to build RaycastVehicle"<<endl;

    std::vector<double> vParameters;
    vParameters.resize(29);
    std::vector<double> position;
    std::vector<double> rotation;

    while(pElement)
    {
      string sAttrName = pElement->Name();

      // Car paramters
      if(!sAttrName.compare("param"))
      {
        std::string param = pElement->Attribute("name");
        if(!param.compare("control delay")){
          vParameters[6] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("stiffness")){
          vParameters[12] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("susp conn height")){
          vParameters[11] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("max susp force")){
          vParameters[13] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("damp factor")){
          vParameters[16] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("exp damp factor")){
          vParameters[17] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("roll influence")){
          vParameters[18] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("steering coeff")){
          vParameters[19] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("max steering")){
          vParameters[20] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("max steering rate")){
          vParameters[21] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("accel offset")){
          vParameters[22] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("steering offset")){
          vParameters[23] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("stall torque coeff")){
          vParameters[24] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("torque speed slope")){
          vParameters[25] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("susp rest length")){
          vParameters[15] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("max susp travel")){
          vParameters[14] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("Magic B")){
          vParameters[26] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("Magic C")){
          vParameters[27] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!param.compare("Magic E")){
          vParameters[28] = GenNumFromChar(pElement->Attribute("value")).front();
        }
      }

      // Vehicle body parameters
      else if(!sAttrName.compare("body")){
        std::string body = pElement->Attribute("name");
        if(!body.compare("length")){
          vParameters[0] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!body.compare("width")){
          vParameters[1] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!body.compare("height")){
          vParameters[2] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!body.compare("mass")){
          vParameters[7] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!body.compare("position")){
          position = GenNumFromChar(pElement->Attribute("value"));
        }
        if(!body.compare("rotation")){
          rotation = GenNumFromChar(pElement->Attribute("value"));
        }

      }

      // Vehicle wheel parameters
      else if(!sAttrName.compare("wheel")){
        std::string wheel = pElement->Attribute("name");
        if(!wheel.compare("radius")){
          vParameters[8] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!wheel.compare("width")){
          vParameters[9] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!wheel.compare("dyn friction")){
          vParameters[3] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!wheel.compare("slip coeff")){
          vParameters[5] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!wheel.compare("traction friction")){
          vParameters[10] = GenNumFromChar(pElement->Attribute("value")).front();
        }
        if(!wheel.compare("side friction")){
          vParameters[4] = GenNumFromChar(pElement->Attribute("value")).front();
        }
      }

      pElement=pElement->NextSiblingElement();
    }

    Eigen::Vector6d dPose;
    dPose<<0,0,0,0,0,1.5707;
    RaycastVehicle* pRaycastVehicle = new RaycastVehicle(sRobotName,vParameters,dPose);

    /// Build the car here.
    m_mModelNodes[sRobotName] = pRaycastVehicle;

    cout<<"[URDF Parser] Parse Car "<<sRobotName<<" Success."<<endl;
  }

  //////////////////////////////////////////

  else if(!sRobotType.compare("Compound"))
  {
    // Need to do something with m_mModelNodes; right now, nothing happens.
    // Which is silly.
    while (pElement){
      const char* sRootContent = pElement->Name();

      //////////////////////////////////////////
      // THE BODY BASE
      //////////////////////////////////////////

      if(strcmp(sRootContent,"bodybase")==0){
        string sBodyName = GetAttribute( pElement, "name")+"@"+sRobotName;
        int iMass =::atoi( pElement->Attribute("mass"));
        vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
        vector<double> vDimesion= GenNumFromChar(pElement->Attribute("dimesion"));
        const char* sType = pElement->Attribute("type");
        if(strcmp(sType, "Box") ==0){
          BoxShape* pBox =new BoxShape(sBodyName, vDimesion[0],vDimesion[1],vDimesion[2],iMass, 1, vPose);
          m_RobotModel.SetBase( pBox ); // main body
          cout<<"[Parse Robot] Build Bodybase: "<<sBodyName<<" success."<<endl;
          m_mModelNodes[pBox->GetName()] = pBox;
        }
      }

      //////////////////////////////////////////
      // ALL OF OUR BODIES (SHAPES)
      //////////////////////////////////////////

      // Build shapes connected to the body base.
      if(strcmp(sRootContent,"body")==0)
      {
        string sBodyName = GetAttribute( pElement, "name")+"@"+sRobotName;
        cout<<"Try to build body "<<sBodyName<<endl;
        int iMass =::atoi( pElement->Attribute("mass"));
        vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
        vector<double> vDimesion =
            GenNumFromChar(pElement->Attribute("dimesion"));
        const char* sType = pElement->Attribute("type");
        if(strcmp(sType, "Box") ==0){
          BoxShape* pBox =new BoxShape(sBodyName,vDimesion[0],vDimesion[1],
              vDimesion[2],iMass, 1,vPose);
          m_mModelNodes[pBox->GetName()] = pBox;
        }
        else if(strcmp(sType,"Cylinder")==0){
          CylinderShape* pCylinder =new CylinderShape(sBodyName,vDimesion[0],
              vDimesion[1],iMass,1,
              vPose);
          m_mModelNodes[pCylinder->GetName()] = pCylinder;
        }
        cout<<"[RobotParser] Finish building body "<<sBodyName<<endl;
      }

      //////////////////////////////////////////
      // ALL OF OUR SENSORS BODIES
      //////////////////////////////////////////

      if(strcmp(sRootContent,"Sensor")==0){
        string sType( pElement->Attribute("Type"));
        if(sType == "Camera"){
          cout<<"[ParseRobot] Try to Init Camera"<<endl;
          const char* sMode = pElement->Attribute("Mode");

          /// Single Camera
          if(strcmp(sMode, "RGB")==0 || strcmp(sMode,"Depth")==0 ||
             strcmp(sMode,"Gray")==0){
            cout<<"[ParseRobot] Try to init "<<sMode<<endl;
            string sCameraName= GetAttribute( pElement, "Name")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy
            string sParentName= GetAttribute( pElement, "Parent")+"@"+sRobotName; // name of body that the sensor attach to. e.g. chassis@robot1@proxy
            string sCamMode(sMode);
            string sSensorName = sCamMode + sCameraName; // this is body name for sensor of the camera. e.g. RGBLCam@robot1@proxy
            vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));
            int iMass = 1;

            // create body for simcam
            vPose[1] = vPose[1]+2; vPose[2] = vPose[2] -3.2;
            BoxShape* pBox =new BoxShape(sSensorName,0.1,0.1,0.1,iMass,1,vPose);
            m_mModelNodes[pBox->GetName()] = pBox;

            // create joint for SimCam
            string sJointName = "SimCamJoint"+sCameraName; // e.g. SimCamJointLCam@robot1@proxy
            Eigen::Vector3d vPivot;
            Eigen::Vector3d vAxis;
            vPivot<<vPose[0],vPose[1]+2,vPose[2]-3.2;
            vAxis<<1,1,1;
            HingeTwoPivot* pHinge =
                new HingeTwoPivot(sJointName,
                                  dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
                                  dynamic_cast<Shape*>(m_mModelNodes.find(sSensorName)->second),
                                  vPivot, Eigen::Vector3d::Identity(),
                                  vAxis, Eigen::Vector3d::Identity());
            m_mModelNodes[pHinge->GetName()] = pHinge;
            cout<<"[ParseRobot] Init Single Camera Physics Success."<<endl;
          }

          /// RGB-Depth Camera
          if(strcmp(sMode, "RGBD")==0 ){
            cout<<"[ParseRobot] Try to init "<<sMode<<endl;
            string sCameraName=
                GetAttribute( pElement, "Name")+"@"+sRobotName;
            string sParentName=
                GetAttribute( pElement, "Parent")+"@"+sRobotName;
            vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));
            int iMass = 1;
            double BodyDistance = 0.5;

            // TODO: Why are we assigning poses here?

            /// CREATE CAMERA BODIES

            // 1.3 create body to connect RGBD Cameras
            vector<double> vCameraPose;
            vCameraPose.push_back(0);vCameraPose.push_back(-2.1);vCameraPose.push_back(0);
            vCameraPose.push_back(0);vCameraPose.push_back(0);vCameraPose.push_back(0);
            BoxShape* pBox = new BoxShape(sCameraName,BodyDistance,0.1,0.1,iMass, 1, vCameraPose);
            m_mModelNodes[pBox->GetName()] = pBox;

            // 1.2 create body for Depth Cam
            string sDepthBodyName = "Depth"+sCameraName;
            vector<double> vDepthCameraPose;
            vDepthCameraPose.push_back(vPose[0]+BodyDistance+0.1);
            vDepthCameraPose.push_back(vPose[1] + 2);
            vDepthCameraPose.push_back(vPose[2]);
            vDepthCameraPose.push_back(0);
            vDepthCameraPose.push_back(0);
            vDepthCameraPose.push_back(0);
            BoxShape* pDepthBox = new BoxShape(sDepthBodyName, 0.1, 0.1, 0.1,
                                               iMass, 1, vDepthCameraPose);
            m_mModelNodes[pDepthBox->GetName()] = pDepthBox;

            // 1.1 create body for RGB Cam
            string sRGBBodyName = "RGB"+sCameraName;
            vector<double> vRGBCameraPose;
            vRGBCameraPose.push_back(vPose[0]-BodyDistance - 0.1);
            vRGBCameraPose.push_back(vPose[1] + 2);
            vRGBCameraPose.push_back(vPose[2]);
            vRGBCameraPose.push_back(0);
            vRGBCameraPose.push_back(0);
            vRGBCameraPose.push_back(0);

            BoxShape* pRGBbox =new BoxShape(sRGBBodyName, 0.1, 0.1, 0.1,
                                            iMass, 1, vRGBCameraPose);
            m_mModelNodes[pRGBbox->GetName()] = pRGBbox;

            /// create joints
            Eigen::Vector3d vPivot;
            Eigen::Vector3d vAxis;
            string sJointName = "SimCamJoint"+sCameraName;
            vPivot<< 0, 0, 0;
            vAxis<<0,-1,0;
            cout<<"before hinge joint."<<endl;
            HingeTwoPivot* pRGBDHinge =
                new HingeTwoPivot(sJointName,
                                  dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
                                  dynamic_cast<Shape*>(pBox),
                                  vPivot, Eigen::Vector3d::Identity(),
                                  vAxis, Eigen::Vector3d::Identity());
            m_mModelNodes[pRGBDHinge->GetName()] = pRGBDHinge;

            // 2.1 create joint for RGB body and RGBDCamBody
            string sRGBJointName = "SimCamJoint"+sRGBBodyName;
            vPivot<<-0.5, 0, 0;
            vAxis<< 1,0,0;
            HingeTwoPivot* pRGBHinge =
                new HingeTwoPivot(sRGBJointName,
                                  dynamic_cast<Shape*>(m_mModelNodes.find(sCameraName)->second),
                                  dynamic_cast<Shape*>(m_mModelNodes.find(sRGBBodyName)->second),
                                  vPivot, Eigen::Vector3d::Identity(),
                                  vAxis, Eigen::Vector3d::Identity());
            m_mModelNodes[pRGBHinge->GetName()] = pRGBHinge;

            // 2.2 create joint for Depth body and RGBDCamBody
            string sDepthJointName = "SimCamJoint"+sDepthBodyName;
            vPivot<< 0.5, 0, 0;
            vAxis<<  1, 0, 0;
            HingeTwoPivot* pDepthHinge =
                new HingeTwoPivot( sDepthJointName, pBox, pDepthBox,
                                   vPivot, Eigen::Vector3d::Identity(),
                                   vAxis, Eigen::Vector3d::Identity());
            m_mModelNodes[pDepthHinge->GetName()] = pDepthHinge;


            for( std::map<string, ModelNode*>::iterator it = m_mModelNodes.begin();
                 it!=m_mModelNodes.end();it++){
              cout<<"find camera body "<<it->first<<endl;
            }
            cout<<"[ParseRobot] Init RGBD Camera Physic Success."<<endl;
          }
          cout<<"[ParseRobot] init Camera Physic Success."<<endl;
        }

        //        if(sType=="GPS")
        //        {
        //          string sBodyName = GetAttribute( pElement, "Name")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy
        //          string sParentName= GetAttribute( pElement, "Parent")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy
        //          vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));
        //          int iMass = 1;
        //          // string sMeshdir(pElement->Attribute("Dir"));

        //          // create body for it
        //          BoxShape* pBox =new BoxShape(sBodyName, 0.1,0.1,0.1,iMass, 1, vPose);
        //          m_mModelNodes.insert(std::pair<std::string, Shape*>(sBodyName,pBox));

        //          // create joint for SimGPS
        //          string sJointName = "GPSJoint"+sBodyName;
        //          Eigen::Vector3d vPivot;
        //          Eigen::Vector3d vAxis;
        //          vPivot<<vPose[0],vPose[1],vPose[2];
        //          vAxis<<1,0,0;
        //          //HingeJoint* pHinge = new HingeJoint( sJointName, m_mModelNodes.find(sParentName)->second, m_mModelNodes.find(sBodyName)->second, vPivot[0], vPivot[1], vPivot[2], vAxis[0],vAxis[1],vAxis[2],100,100,0,M_PI );
        //        }

      }

      //////////////////////////////////////////
      // ALL OF OUR JOINTS (CONSTRAINTS)
      //////////////////////////////////////////

      if(strcmp(sRootContent,"joint")==0){
        cout<<"[Parse Robot] try to build joint"<<endl;
        string sJointName= GetAttribute( pElement, "name")+"@"+sRobotName; // get joint name. e.g. BLAxleJoint@robot1@proxy
        string sJointType(pElement->Attribute("type"));
        if(sJointType == "HingeJoint"){
          cout<<"[Parse Robot] try to build Hinge joint."<<endl;
          string sParentName;
          string sChildName;
          vector<double> vPivot;
          vector<double> vAxis;
          double dUpperLimit = M_PI;
          double dLowerLimit = 0;
          double dDamping = 1;
          double dStiffness = 1;

          // Construct joint based on children information. This information may include links, origin, axis.etc
          XMLElement *pChild=pElement->FirstChildElement();
          while(pChild){
            const char * sName = pChild->Name();
            // get parent body of joint
            if(strcmp(sName, "parent")==0){
              sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
            }

            // get child body of joint
            if(strcmp(sName, "child")==0){
              sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
            }
            if(strcmp(sName, "pivot")==0){
              vPivot=GenNumFromChar( pChild->Attribute("setting"));
            }
            if(strcmp(sName, "axis")==0){
              vAxis=GenNumFromChar( pChild->Attribute("setting"));
            }
            if(strcmp(sName,"upperlimit")==0){
              dUpperLimit = ::atof(sName);
            }
            if(strcmp(sName,"lowerlimit")==0){
              dLowerLimit = ::atof(sName);
            }
            if(strcmp(sName,"damping")==0){
              dDamping = ::atof(sName);
            }
            if(strcmp(sName,"stiffness")==0){
              dStiffness = ::atof(sName);
            }
            // read next child (joint)
            pChild=pChild->NextSiblingElement();
          }
          Eigen::Vector3d pivot;
          pivot<<vPivot[0], vPivot[1], vPivot[2];
          Eigen::Vector3d axis;
          axis<<vAxis[0], vAxis[1], vAxis[2];
          HingeTwoPivot* pHinge =
              new HingeTwoPivot( sJointName,
                                 dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
                                 dynamic_cast<Shape*>(m_mModelNodes.find(sChildName)->second),
                                 pivot, Eigen::Vector3d::Identity(),
                                 axis, Eigen::Vector3d::Identity());
          m_mModelNodes[pHinge->GetName()] = pHinge;
          cout<<"Build Hinge joint "<<sJointName<<" success"<<endl;
        }
        else if(sJointType=="Hinge2Joint")
        {
          cout<<"[Parse Robot] try to build Hinge2 joint."<<endl;
          string sParentName;
          string sChildName;
          vector<double> vAnchor;
          vector<double> vAxis1;
          vector<double> vAxis2;
          vector<double> vLowerLinearLimit;
          vector<double> vUpperLinearLimit;
          vector<double> vLowerAngleLimit;
          vector<double> vUpperAngleLimit;

          Eigen::Vector3d Anchor;
          Eigen::Vector3d Axis1;
          Eigen::Vector3d Axis2;
          Eigen::Vector3d LowerLinearLimit;
          Eigen::Vector3d UpperLinearLimit;
          Eigen::Vector3d LowerAngleLimit;
          Eigen::Vector3d UpperAngleLimit;

          // read detail of a joint
          XMLElement *pChild=pElement->FirstChildElement();

          // Construct joint based on children information. This information may include links, origin, axis.etc
          while(pChild){
            const char * sName = pChild->Name();
            // get parent link of joint
            if(strcmp(sName, "parent")==0){
              sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
            }
            // get child link of joint
            if(strcmp(sName, "child")==0){
              sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
            }
            if(strcmp(sName, "anchor")==0){
              vAnchor = GenNumFromChar( pChild->Attribute("setting"));
              Anchor<<vAnchor[0],vAnchor[1],vAnchor[2];
            }
            if(strcmp(sName, "axis")==0){
              vAxis1 = GenNumFromChar( pChild->Attribute("axis1"));
              vAxis2 = GenNumFromChar( pChild->Attribute("axis2"));

              Axis1<<vAxis1[0],vAxis1[1],vAxis1[2];
              Axis2<<vAxis2[0],vAxis2[1],vAxis2[2];
            }
            if(strcmp(sName, "limit")==0){
              vLowerLinearLimit = GenNumFromChar( pChild->Attribute("lowerlinear"));
              vUpperLinearLimit = GenNumFromChar( pChild->Attribute("upperlinear"));
              vLowerAngleLimit = GenNumFromChar( pChild->Attribute("lowerangle"));
              vUpperAngleLimit = GenNumFromChar( pChild->Attribute("upperangle"));
              LowerAngleLimit<<vLowerAngleLimit[0],vLowerAngleLimit[1],vLowerAngleLimit[2];
              UpperAngleLimit<<vUpperAngleLimit[0],vUpperAngleLimit[1],vUpperAngleLimit[2];
              LowerLinearLimit<<vLowerLinearLimit[0],vLowerLinearLimit[1],vLowerLinearLimit[2];
              UpperLinearLimit<<vUpperLinearLimit[0],vUpperLinearLimit[1],vUpperLinearLimit[2];
            }
            // read next child (joint)
            pChild=pChild->NextSiblingElement();
          }

          cout<<"before build joint"<<endl;
          if(m_mModelNodes.find(sParentName) !=m_mModelNodes.end()){
            cout<<"find "<<sParentName<<endl;
          }
          else{
            cout<<"Cannot find"<<sParentName<<endl;
          }
          for(std::map<string, ModelNode*>::iterator it =m_mModelNodes.begin();
              it!=m_mModelNodes.end(); it++ ){
            cout<<"find "<<it->first<<endl;
          }
          Hinge2* pHinge2 = new Hinge2( sJointName,
                                        dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
                                        dynamic_cast<Shape*>(m_mModelNodes.find(sChildName)->second),
                                        Anchor, Axis1, Axis2);
          //TODO:
          //pHinge2->SetLimits(1, 1, LowerLinearLimit, UpperLinearLimit,
          //LowerAngleLimit, UpperAngleLimit);
          m_mModelNodes[pHinge2->GetName()] = pHinge2;
          std::cout<<"[ParseRobot] Creating a Hinge2Joint between "<<
                     sParentName<<" and "<<sChildName<<std::endl;
        }
      }
      // read next parent element
      pElement=pElement->NextSiblingElement();
    }
  }
  else{
    std::cerr<<"[] I have no idea what I'm doing"<<std::endl;
    return false;
  }

  m_RobotModel.SetParts(GetModelNodes());
  cout<<"[ParseRobot] Finish"<<endl;
  return true;
}



////////////////////////////////////////////////////////////
/// PARSE ROBOT.XML FOR DEVICES AND BUILD INTO ROBOTPROXY
/// Extract all devices and build vSimDeviceInfo.
////////////////////////////////////////////////////////////

bool URDF_Parser::ParseDevices(
    XMLDocument& doc, vector<SimDeviceInfo>&  m_vSimDeviceInfo,
    string sProxyName)
{
  XMLElement *pParent=doc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();
  string sRobotName(GetAttribute(pParent,"name"));
  sRobotName = sRobotName+"@"+sProxyName;          // e.g. robo

  // read high level parent (root parent)
  while (pElement)
  {
    const char* sRootContent = pElement->Name();

    // create sim Sensor device
    if(strcmp(sRootContent,"Sensor")==0)
    {
      string sType( pElement->Attribute("Type"));

      if(sType == "Camera")
      {
        const char* sMode = pElement->Attribute("Mode");
        string sModel = GetAttribute(pElement, "Model");

        //----------------------------------------------------------------------------------------- Singel Camera
        if(strcmp(sMode, "RGB")==0 || strcmp(sMode,"Depth")==0 ||strcmp(sMode,"Gray")==0)
        {
          string sCameraName= GetAttribute( pElement, "Name")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy
          string sCamMode(sMode);
          string sSensorName = sCamMode + sCameraName; // this is body name for sensor of the camera. e.g. RGBLCam@robot1@proxy
          int iFPS=atoi( GetAttribute(pElement,"FPS").c_str());
          vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));

          // save device info
          SimDeviceInfo Device;
          Device.m_sDeviceName = sCameraName;
          Device.m_sDeviceType = sType;
          Device.m_iFPS = iFPS;
          Device.m_vSensorList.push_back(sSensorName);
          Device.m_vModel.push_back(sModel);
          Device.m_vPose<<vPose[0],vPose[1],vPose[2],vPose[3],vPose[4],vPose[5];
          m_vSimDeviceInfo.push_back(Device);

          cout<<"[Proxy/ParseDevice] register "<<sType<<" (SimCam "<<sSensorName<<") success. Device Name is "<<sCameraName<<"."<<endl;
        }

        // ---------------------------------------------------------------------------------------- RGB-Depth Camera
        if(strcmp(sMode, "RGBD")==0 )
        {
          string sCameraName= GetAttribute( pElement, "Name")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy
          string sRGBBodyName = "RGB"+sCameraName;
          string sDepthBodyName = "Depth"+sCameraName;
          int iFPS=atoi( GetAttribute(pElement,"FPS").c_str());
          vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));

          // 3 save intp device, this device have two sensors
          SimDeviceInfo Device;
          Device.m_sDeviceName = sCameraName;
          Device.m_sDeviceType = sType;
          Device.m_iFPS = iFPS;
          Device.m_vSensorList.push_back(sRGBBodyName);
          Device.m_vSensorList.push_back(sDepthBodyName);
          Device.m_vModel.push_back(sModel);
          Device.m_vModel.push_back(sModel);
          Device.m_vPose<<vPose[0],vPose[1],vPose[2],vPose[3],vPose[4],vPose[5];
          m_vSimDeviceInfo.push_back(Device);

          cout<<"[Proxy/ParseDevice] register "<<sType<<" (SimCam "<<sMode<<") success." <<endl;
        }
      }

      if(sType=="GPS")
      {
        string sBodyName = GetAttribute( pElement, "Name")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy

        cout<<"[Proxy/ParseDevice] Register Sim GPS success."<<endl;
      }


      if(sType=="Vicon")
      {
        string sViconName = GetAttribute( pElement, "Name")+"@"+sRobotName;// name of the camera. e.g. LCam@robot1@proxy
        string sBodyName= GetAttribute( pElement, "Body")+"@"+sRobotName;

        SimDeviceInfo Device;
        Device.m_sDeviceName = sViconName;
        Device.m_sDeviceType = sType;
        Device.m_sBodyName = sBodyName;
        m_vSimDeviceInfo.push_back(Device);
        cout<<"[Proxy/ParseDevice] Add vicon device "<<sViconName<<" success."<<endl;
      }
    }

    // create sim controller device
    if(strcmp(sRootContent, "Controller")==0)
    {
      string sType( sRootContent );
      string sMode( pElement->Attribute("Mode"));

      if(sMode == "SimpleController")
      {
        string  sControllerName = GetAttribute(pElement, "Name");

        SimDeviceInfo Device;
        Device.m_sDeviceName = sControllerName;
        Device.m_sDeviceType = sType;
        Device.m_sDeviceMode = sMode;
        Device.m_sRobotName = sRobotName;
        m_vSimDeviceInfo.push_back(Device);
        cout<<"[Proxy/ParseDevice] Add controller device "<<sControllerName<<" success."<<endl;
      }

      if(sType =="CarController")
      {


      }
    }

    // read next parent element
    pElement=pElement->NextSiblingElement();
  }
}


////////////////////////////////////////////////////////////////////////////
/// PARSE WORLD.XML FOR STATEKEEPER
////////////////////////////////////////////////////////////////////////////

bool URDF_Parser::ParseWorldForInitialPoses(
    const char* filename,
    vector<Eigen::Vector6d>& vRobotInitPose){

  // make sure the vector is empty
  vRobotInitPose.clear();

  // open xml document
  XMLDocument doc;
  if(doc.LoadFile(filename) !=0){
    printf("Cannot open %s\n", filename);
    return false;
  }

  XMLElement *pParent=doc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();

  // read high level parent (root parent)
  while (pElement){
    const char* sRootContent = pElement->Name();
    if(strcmp(sRootContent,"robot")==0){
      vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
      Eigen::Vector6d ePose;
      ePose<<vPose[0], vPose[1], vPose[2], vPose[3], vPose[4], vPose[5];
      vRobotInitPose.push_back(ePose);
    }
    pElement=pElement->NextSiblingElement();
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////
/// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////

std::vector<ModelNode*> URDF_Parser::GetModelNodes(){
  std::vector<ModelNode*> Nodes;
  for( std::map<string, ModelNode*>::iterator it = m_mModelNodes.begin();
       it!=m_mModelNodes.end();it++){
    Nodes.push_back(it->second);
  }
  return Nodes;
}



