#include <Managers/NetworkManager.h>

/************************************************************
 *
 * INITIALIZER
 *
 ***********************************************************/

bool NetworkManager::Init(string sProxyName, string sServerName,
                          int debug_level) {
  server_name_ = sServerName;
  debug_level_ = debug_level;
  if (server_name_ == "WithoutNetwork") {
    LOG(debug_level_) << "Skip Network due to WithoutNetwork Mode";
    return true;
  }
  // If we do need it, Init Node connection in LocalSim and provide
  // relative RPC method.
  local_sim_name_  = sProxyName;
  m_iNodeClients   = 0;
  node_.set_verbosity(1);
  bool worked = node_.init(local_sim_name_);
  if (!worked) {
    LOG(debug_level_) << "FAILURE: No init for node " << local_sim_name_;
    return false;
  }
  LOG(debug_level_) << "SUCCESS: Initialized node for " << local_sim_name_;
  node_.provide_rpc("AddRenderObject", &_AddRenderObject, this);
  LOG(debug_level_) << "SUCCESS: rpc call for adding render objects"
                    << " [AddRenderObject] registered with Node";
  return true;
}

/************************************************************
 *
 * URI PARSERS
 * These are incredibly useful; they provide parameters,
 * and they're the only things that tell us what
 * devices the user wants on or off.
 *
 ***********************************************************/

std::map<string, string> NetworkManager::ParseURI(string sURI) {
  std::map<string, string> uri_contents;
  std::size_t found = sURI.find(":");
  if (found!=std::string::npos) {
    uri_contents.insert(std::pair<string,string>("device",
                                                 sURI.substr(0, found)));
    found = found+2;
    sURI = sURI.substr(found);
  }
  while (sURI.find(",") != std::string::npos) {
    std::size_t found_name = sURI.find("=");
    std::size_t found_value = sURI.find(',');
    uri_contents.insert(std::pair<string,string>(
        sURI.substr(0, (found_name)),
        sURI.substr(found_name+1,
                    (found_value-found_name-1))));
    sURI = sURI.substr(found_value+1);
  }
  std::size_t found_name = sURI.find("=");
  std::size_t found_value = sURI.find(']');
  uri_contents.insert(std::pair<string,string>(
      sURI.substr(0, (found_name)),
      sURI.substr(found_name+1,
                  (found_value-found_name-1))));
  LOG(debug_level_) << "URI parameters:";
  for (map<string, string>::iterator it = uri_contents.begin();
       it != uri_contents.end();
       it++) {
    LOG(debug_level_) << it->first << " is set to " << it->second;
  }
  return uri_contents;
}

// Return 'FALSE' if device is invalid. Otherwise return device name.

string NetworkManager::CheckURI(string sURI) {
  // Find device in device manager
  std::map<string, string> uri_contents = ParseURI(sURI);
  string sDeviceName = uri_contents["name"]+"@"+uri_contents["sim"];
  vector<SimDeviceInfo*> pDevices =
      sim_devices_->GetAllRelatedDevices(sDeviceName);
  if (!pDevices.empty()) {
    // Check for different devices here. Not sure how else to do this...
    if (uri_contents["device"] == "openni") {
      if (uri_contents["rgb"] == "1") {
        sim_devices_->sim_device_map_["RGB_"+sDeviceName]->m_bDeviceOn = true;
      }
      if (uri_contents["depth"] == "1") {
        sim_devices_->sim_device_map_["Depth_"+sDeviceName]->m_bDeviceOn = true;
      }
    }
    /// NodeCar controller
    if (uri_contents["device"] == "NodeCar") {
      sim_devices_->sim_device_map_[sDeviceName]->m_bDeviceOn = true;
      LOG(debug_level_) << "Got a NodeCar Controller!";
    }
    return sDeviceName;
  }
  return "FALSE";
}

/************************************************************
 *
 * NODE FUNCTIONS
 *
 ***********************************************************/

////////////////////////////////////////////////////////////////////////
// Initializes all devices attached to the robot into Node.
// Notice that if we run in 'WithStateKeeper' mode, we need to call this
// function after the main robot has gotten its initial pose from StateKeeper,
// and been built into the ModelGraph.

void NetworkManager::RegisterDevices(
    const std::shared_ptr<SimDevices> pSimDevices) {
  sim_devices_ = pSimDevices;
  // Check if we need to init devices in Node.
  if (server_name_ == "WithoutNetwork") {
    LOG(debug_level_) << "Skip! Init LocalSim without Network.";
    // Turn all of our devices on, since it doesn't matter.
    for (map<string, SimDeviceInfo*>::iterator it =
             sim_devices_->sim_device_map_.begin();
         it != sim_devices_->sim_device_map_.end();
         it++) {
      SimDeviceInfo* Device = it->second;
      Device->m_bDeviceOn = true;
    }
  } else {
    for (map<string, SimDeviceInfo*>::iterator it =
             sim_devices_->sim_device_map_.begin();
         it != sim_devices_->sim_device_map_.end();
         it++) {
      SimDeviceInfo* Device = it->second;

      /*******************
       * SimSensors
       ********************/

      /// CAMERAS
      if (Device->m_sDeviceType=="Camera" && !Device->m_bHasAdvertised) {
        vector<SimDeviceInfo*> related_devices =
            sim_devices_->GetAllRelatedDevices(Device->GetBodyName());
        SimCamera* pCam = (SimCamera*) related_devices.at(0);
        // provide rpc method for camera to register
        node_.provide_rpc("RegisterSensorDevice",&_RegisterSensorDevice,this);
        LOG(debug_level_) << "SUCCESS: rpc call for Camera(s) "
                          << "[RegisterSensorDevice] registered with Node";
        for (unsigned int ii = 0; ii < related_devices.size(); ii++) {
          related_devices.at(ii)->m_bHasAdvertised = true;
        }
      }
      /// GPS
      //      else if (Device->m_sDeviceType=="GPS" &&
      //               !Device->m_bHasAdvertised) {
      //        vector<SimDeviceInfo*> related_devices =
      //            sim_devices_->GetAllRelatedDevices(Device->GetBodyName());
      //        SimGPS* pGPS = (SimGPS*) related_devices.at(0);
      //        pGPS->m_bDeviceOn = true;
      //        node_.advertise(pGPS->GetDeviceName());
      //      }
      //      /// VICON
      //      else if (static_cast<SimVicon*>(Device) != NULL) {
      //        SimVicon* pVicon = (SimVicon*) Device;
      //        pVicon->m_bDeviceOn = true;
      //        pVicon->Update();
      //      }

      /*******************
       * SimControllers
       ********************/

      else if (Device->m_sDeviceType == "CarController"
               && !Device->m_bHasAdvertised) {
        CarController* pCarCon = (CarController*) Device;
        node_.provide_rpc("RegisterControllerDevice",
                          &_RegisterControllerDevice, this);
        LOG(debug_level_) << "SUCCESS: rpc call for CarController "
                          << "[RegisterControllerDevice] registered with Node";
        pCarCon->m_bHasAdvertised = true;
      }

      //      else if (static_cast<SimpleController*>(Device) != NULL) {
      //        SimpleController* pSimpleCon = (SimpleController*) Device;
      //        pSimpleCon->m_bDeviceOn = true;
      //      }
    }
  }
}

////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE DEVICES FROM THE SIMULATION

void NetworkManager::RegisterSensorDevice(RegisterNodeCamReqMsg& mRequest,
                                          RegisterNodeCamRepMsg & mReply) {
  LOG(debug_level_) << "NodeCam asking for register in timestep "
                    << timestep_ << ".";
  string sDeviceName = CheckURI(mRequest.uri());
  if (sDeviceName != "FALSE") {
    vector<SimDeviceInfo*> pDevices =
        sim_devices_->GetOnRelatedDevices(sDeviceName);

    /// TODO: Get all of the devices in this function!!

    SimCamera* pCam = (SimCamera*) pDevices.at(0);
    // For multiple-camera systems, we take the parameters from
    // the first camera.
    mReply.set_time_step(timestep_);
    mReply.set_regsiter_flag(1);
    mReply.set_channels(pDevices.size());
    mReply.set_width(pCam->image_width_);
    mReply.set_height(pCam->image_height_);
    m_iNodeClients = m_iNodeClients + 1;
    string sServiceName = GetFirstName(pCam->GetBodyName());
    if (node_.advertise(sServiceName) == true) {
      LOG(debug_level_) << "SUCCESS: Advertising " << sServiceName;
    } else {
      LOG(debug_level_) << "FAILURE: Advertising " << sServiceName;
      exit(-1);
    }
  }
  else{
    mReply.set_time_step(timestep_);
    mReply.set_regsiter_flag(0);
    LOG(debug_level_) << "HAL device named " << sDeviceName << " is invalid!";
  }
}

////////////////////////////////////////////////////////////////////////

void NetworkManager::RegisterControllerDevice(
    pb::RegisterControllerReqMsg& mRequest,
    pb::RegisterControllerRepMsg & mReply) {
  string controller_name = GetFirstName(CheckURI(mRequest.uri()));
  if (controller_name!="FALSE") {
    int subscribe_try = 0;
    while (node_.subscribe(controller_name + "/" + controller_name) != true &&
           subscribe_try < 500) {
      LOG(debug_level_) << ".";
      subscribe_try++;
    }
    if (subscribe_try >= 500) {
      LOG(debug_level_) << "FAILURE: Cannot subscribe to "<<controller_name;
    } else {
      LOG(debug_level_) << "SUCCESS: Subcribed to " << controller_name;
      mReply.set_success(true);
    }
  } else {
    LOG(ERROR) << "FAILURE: Controller's URI could not be parsed";
  }
  LOG(debug_level_) << "SUCCESS: Done adding CarController";
}

////////////////////////////////////////////////////////////////////////

void NetworkManager::AddRenderObject(pb::RegisterRenderReqMsg& mRequest,
                                     pb::RegisterRenderRepMsg& mReply) {
  LOG(debug_level_) << "AddRenderObject called through RPC...";
  LOG(debug_level_) << "Adding shapes to Scene.";
  pb::SceneGraphMsg new_objects = mRequest.new_objects();
  std::vector<std::shared_ptr<ModelNode> > new_parts;
  if (new_objects.has_box()) {
    auto box_info = new_objects.box();
    std::vector<double> pose;
    pose.resize(6);
    string name = box_info.name();
    double x_length = box_info.x_length();
    double y_length = box_info.y_length();
    double z_length = box_info.z_length();
    double mass = box_info.mass();
    for (int ii = 0; ii < box_info.pose().size(); ii++) {
      pose.at(ii) = box_info.pose().Get(ii);
    }
    std::shared_ptr<BoxShape> pBox = std::make_shared<BoxShape>(
        name, x_length, y_length, z_length,
        mass, 1, pose);
    new_parts.push_back(pBox);
    LOG(debug_level_) << "    Box added";
  }
  // else if (new_objects.has_cylinder) {
  //   CylinderShape* pCylinder =new CylinderShape(sBodyName, vDimension[0],
  //                                               vDimension[1], iMass,1,
  //                                               vPose);
  //   new_parts.push_back(pCylinder);
  // } else if (new_objects.has_sphere) {
  //   SphereShape* pSphere =new SphereShape(sBodyName, vDimension[0],
  //                                         iMass, 1, vPose);
  //   new_parts.push_back(pSphere);
  // } else if (new_objects.has_plane) {
  //   PlaneShape* pPlane =new PlaneShape(sBodyName, vDimension, vPose);
  //   new_parts.push_back(pPlane);
  // } else if (new_objects.has_mesh) {
  //   string file_dir = pElement->Attribute("dir");
  //   MeshShape* pMesh =new MeshShape(sBodyName, file_dir, vPose);
  //   new_parts.push_back(pMesh);
  // } else if (new_objects.has_light) {
  //   LightShape* pLight = new LightShape("Light", vLightPose);
  //   new_parts.push_back(pLight);
  // }
  else if (new_objects.has_waypoint()) {
    auto waypoint_info = new_objects.waypoint();
    string name = waypoint_info.name();
    std::vector<double> pose;
    pose.resize(6);
    for (int ii = 0; ii < waypoint_info.pose().size(); ii++) {
      pose.at(ii) = waypoint_info.pose().Get(ii);
    }
    double velocity = waypoint_info.velocity();
    std::shared_ptr<WaypointShape> pWaypoint = std::make_shared<WaypointShape>(
        name, pose, velocity);
    new_parts.push_back(pWaypoint);
    LOG(debug_level_) << "    Waypoint added";
  }
  SimRobot* rob = robot_manager_->GetMainRobot();
  rob->SetParts(new_parts);
  LOG(debug_level_) << "SUCCESS: SceneGraph shapes added.";
}


////////////////////////////////////////////////////////////////////////
/// UPDATE AND PUBLISH INFO
////////////////////////////////////////////////////////////////////////

bool NetworkManager::UpdateNetwork() {
  timestep_++;
  /// -s WithoutNetwork
  if (server_name_ == "WithoutNetwork") {
    return true;
  }
  /// -s WithoutStateKeeper
  ///  N.B. = A sensor must be published by its BODY NAME, called with
  ///         sensor->GetBodyName(). This is because one body can have
  ///         more than one device on it, and so we must account for them
  ///         all.
  ///         A controller must be called by its DEVICE NAME, called with
  ///         controller->GetDeviceName(). It doesn't have a physics body;
  ///         rather, it just controls one. It's purely a device.
  vector<SimDeviceInfo*> pDevices = sim_devices_->GetOnDevices();
  for (unsigned int ii = 0; ii < pDevices.size(); ii++) {
    SimDeviceInfo* Device = pDevices.at(ii);
    ////////////////////
    // SENSORS
    //-- Camera
    if (Device->m_sDeviceType == "Camera" && !Device->m_bHasPublished) {
      PublishSimCamBySensor(Device->GetBodyName());
    }
    //-- GPS
    else if (Device->m_sDeviceType == "GPS" && !Device->m_bHasPublished) {
      PublishGPS(Device->GetBodyName());
    }
    ////////////////////
    // CONTROLLERS
    //-- CarController
    else if (Device->m_sDeviceType == "CarController" &&
             !Device->m_bHasPublished) {
      ReceiveControllerInfo(Device->GetDeviceName());
    }
    Device->m_bHasPublished = false;
  }

  /// -s WithStateKeeper
  if (server_name_ == "WithStateKeeper") {
    LOG(debug_level_) << "Why do you have this switched on?";
  }
  return true;
}

////////////////////////////////////////////////////////////////////////
// Publish SimCamera by Sensor.
// We MUST MAKE SURE that the glGraph has been activated, or else there
// will be nothing to take a picture of!

bool NetworkManager::PublishSimCamBySensor(string sCamBodyName) {
  pb::CameraMsg mCamImage;
  mCamImage.set_device_time(timestep_);
  vector<SimDeviceInfo*> pDevices =
      sim_devices_->GetOnRelatedDevices(sCamBodyName);
  for (unsigned int ii = 0; ii < pDevices.size(); ii++) {
    SimCamera* pSimCam = (SimCamera*) pDevices.at(ii);
    pSimCam->m_bHasPublished = true;

    ////////////
    // A grayscale image
    ////////////
    if (pSimCam->glcamera_type_ == SceneGraph::eSimCamLuminance) {
      pb::ImageMsg *pImage = mCamImage.add_image();
      char* pImgbuf = (char*)malloc (pSimCam->image_width_ *
                                     pSimCam->image_height_);
      if (pSimCam->capture(pImgbuf) == true) {
        pImage->set_timestamp( timestep_);
        pImage->set_width( pSimCam->image_width_ );
        pImage->set_height( pSimCam->image_height_ );
        pImage->set_type(pb::PB_UNSIGNED_SHORT);
        pImage->set_format(pb::PB_LUMINANCE);
        pImage->set_data(pImgbuf);
        LOG(debug_level_) << "Published Greyscale";
      } else {
        return false;
      }
      free(pImgbuf);
    }
    ////////////
    // An RGB image
    ////////////
    else if (pSimCam->glcamera_type_ == SceneGraph::eSimCamRGB) {
      pb::ImageMsg *pImage = mCamImage.add_image();
      char* pImgbuf= (char*)malloc (pSimCam->image_width_ *
                                    pSimCam->image_height_ * 3);
      if (pSimCam->capture(pImgbuf) == true) {
        pImage->set_data(pImgbuf);
        pImage->set_timestamp( timestep_);
        pImage->set_width( pSimCam->image_width_ );
        pImage->set_height( pSimCam->image_height_ );
        pImage->set_type(pb::PB_UNSIGNED_BYTE);
        pImage->set_format(pb::PB_RGB);
        LOG(debug_level_) << "Published RGB";
      } else {
        return false;
      }
      free(pImgbuf);
    }
    ////////////
    // A depth image
    ////////////
    else if (pSimCam->glcamera_type_ == SceneGraph::eSimCamDepth) {
      pb::ImageMsg *pImage = mCamImage.add_image();
      float* pImgbuf = (float*) malloc( pSimCam->image_width_ *
                                        pSimCam->image_height_ *
                                        sizeof(float) );
      if (pSimCam->capture(pImgbuf) == true) {
        pImage->set_data((char*)pImgbuf);
        pImage->set_timestamp( timestep_);
        pImage->set_width(pSimCam->image_width_);
        pImage->set_height(pSimCam->image_height_);
        pImage->set_type(pb::PB_FLOAT);
        pImage->set_format(pb::PB_LUMINANCE);
        LOG(debug_level_) << "Published Depth";
      } else {
        return false;
      }
      free(pImgbuf);
    }
  }

  // Publish the info
  string sFirstName = GetFirstName(sCamBodyName);
  LOG(debug_level_) << "Camera handle to puublish: " << sFirstName;
  bool bStatus = node_.publish(sFirstName, mCamImage);
  if (!bStatus) {
    LOG(ERROR) << "FAILURE: [" << local_sim_name_ << "/" << sFirstName
               << "] cannot publish images"<<endl;
    return false;
  }
  LOG(debug_level_) << "SUCCESS: [" << local_sim_name_ << "/" << sFirstName
                    << "] NodeCam published." <<endl;
  return true;
}

////////////////////////////////////////////////////////////////////////
// publish GPS by device name

bool NetworkManager::PublishGPS(string sDeviceName) {
  Eigen::Vector3d pose;
  SimGPS* pGPS = (SimGPS*) sim_devices_->sim_device_map_[sDeviceName];
  pGPS->GetPose(pose);
  GPSMsg mGPSMSg;
  mGPSMSg.set_time_step(timestep_);
  mGPSMSg.set_x(pose[0]);
  mGPSMSg.set_y(pose[1]);
  mGPSMSg.set_y(pose[2]);
  string sFirstName = GetFirstName(sDeviceName);
  LOG(debug_level_) << "Attempting to publish " << sFirstName
                    << ". x=" << pose[0]
                    << " y=" << pose[1] << " z=" << pose[2]
                    << ". Time step is " << mGPSMSg.time_step() << ".";
  bool bStatus=false;
  while (bStatus == false) {
    bStatus=node_.publish(sFirstName, mGPSMSg);
    if ( bStatus == false) {
      LOG(ERROR) << "FAILURE: Could not publish GPS data.";
    }
  }
  LOG(debug_level_) << "SUCCESS: Published GPS data";
  return true;
}

////////////////////////////////////////////////////////////////////////
// Receive information from all controllers hooked up to HAL.
// (we may have more than one controller here.)

bool NetworkManager::ReceiveControllerInfo(string sDeviceName) {
  SimDeviceInfo* pDevice = sim_devices_->sim_device_map_[sDeviceName];
  /// TODO: There's a naming issue here; we have to duplicate the name
  /// to register correctly.
  string sServiceName = GetFirstName(sDeviceName) + "/"
      + GetFirstName(sDeviceName);
  LOG(debug_level_) << "Attempting to connect to " << sServiceName;

  // ASYNCHRONIZED PROTOCOL: The while loop doesn't wait forever to receive
  //   a command
  // SYNCHRONIZED PROTOCOL: ...we wait.
  bool sync = false;
  LOG(debug_level_) << "Syncing protocol is " << sync;
  int max_iter;
  sync ? max_iter = 1e7 : max_iter = 100;

  // Car Controller
  if (pDevice->m_sDeviceType == "CarController") {
    CarController* pCarController = (CarController*) pDevice;
    pb::VehicleMsg Command;
    int n = 0;
    while (node_.receive(sServiceName, Command)==false
           && n < max_iter) {
      n++;
    }
    if (n == max_iter) {
      LOG(debug_level_) << "No command";
      return false;
    } else {
      LOG(debug_level_) << "Got command!";
      pCarController->UpdateCommand(Command);
      pCarController->m_bHasPublished = true;
      return true;
    }
  }

  // Simple Controller (aka Camera Body control?)
  else if (pDevice->m_sDeviceType == "SimpleController") {
    SimpleController* pSimpController = (SimpleController*) pDevice;
    pb::PoseMsg Command;
    if (node_.receive( sServiceName, Command)==true) {
      pSimpController->UpdateCommand(Command);
      return true;
    } else {
      LOG(ERROR) << "FAILURE: Couldn't get the command for the"
                 << " Simple Controller";
      return false;
    }
  }
  return true;
}
