#include <StateKeeper.h>

using namespace std;

///////////////////////////////////////////////////////
///// CONSTRUCTOR
///////////////////////////////////////////////////////

StateKeeper::StateKeeper(string sStateKeeperName, string sWorldURDFFile)
{
  // initialize node
//  m_Node.set_verbocity(1); // be a bit noisy
  m_Node.init( sStateKeeperName );

  // set up a publisher
  m_Node.advertise( "WorldState" );

  // set up a remote procedure call
  m_Node.provide_rpc("RegisterLocalSim",&_RegisterLocalSim,this);

  // init timestep
  m_iTimeStep=0;

  m_sWorldURDFFileName = sWorldURDFFile;

  cout<<"Init "<<sStateKeeperName<<" success."<<endl;
}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

void StateKeeper::InitRobotPose()
{
  // the is the initial pose of all robot that want to add into StateKeeper.
  URDFParser mParser(1);
  mParser.ParseWorldForInitRobotPose(m_sWorldURDFFileName.c_str(), m_vInitialPose);
}



////////////////////////////////////////////////////////////////////////

void StateKeeper::RegisterLocalSim(RegisterLocalSimReqMsg& mRequest,
                                     RegisterLocalSimRepMsg& mReply){
  // 1. Initialize the robot's pose.
  m_eLastJoinRobotInitPose = m_vInitialPose[0];
  m_vInitialPose.erase(m_vInitialPose.begin());

  mReply.mutable_init_pose()->set_x(m_eLastJoinRobotInitPose[0]);
  mReply.mutable_init_pose()->set_y(m_eLastJoinRobotInitPose[1]);
  mReply.mutable_init_pose()->set_z(m_eLastJoinRobotInitPose[2]);
  mReply.mutable_init_pose()->set_p(m_eLastJoinRobotInitPose[3]);
  mReply.mutable_init_pose()->set_q(m_eLastJoinRobotInitPose[4]);
  mReply.mutable_init_pose()->set_r(m_eLastJoinRobotInitPose[5]);
  mReply.set_robot_name(mRequest.mutable_urdf()->robot_name());
  mReply.set_time_step(m_iTimeStep);


  // 2. reply all urdf file that StateKeeper current hold to new joint proxy
  std::map<string, URDFMsg*>::iterator iter;
  cout<<"[StateKeeper/RegisterLocalSim] Pakcing "<<m_mURDF.size()<<" URDF files to new join proxy "<<mRequest.proxy_name()<<endl;
  for(iter=m_mURDF.begin();iter!=m_mURDF.end();iter++)
  {
    URDFMsg* pPreviousURDF = iter->second;
    string sRobotName = pPreviousURDF->robot_name();
    URDFMsg* pReplyURDF= mReply.add_urdf();
    // pReplyURDF->set_robot_name(pPreviousURDF->robot_name()); // it is very strange that this line doesn't work. need to fix this error in the furture.
    pReplyURDF->set_robot_name(iter->first);
    pReplyURDF->set_xml(pPreviousURDF->xml());
    cout<<"[StateKeeper/RegisterLocalSim] Prepare previous robot URDF with robot name "<<sRobotName<<", key string name "<<iter->first<<" for "<<mRequest.proxy_name()<<endl;
  }


  // 4. subscribe to RobotPorxy's Robot state topic
  string sServiceName= mRequest.proxy_name()+"/RobotState";
  if( m_Node.subscribe(sServiceName) == false )
  {
    cout<<"[StateKeeper/RegisterLocalSim] Error subscribing to "<<sServiceName<<". Forbid client: "<<mRequest.proxy_name()<<"."<<endl;
  }
  else
  {
    cout<<"[StateKeeper/RegisterLocalSim] subscribe to '"<<sServiceName<<"' success."<<endl;
  }

  // 3. send URDF of new client to all other Proxys.
  m_bSendURDFtoProxys = true;
  m_sLastJoinRobotName = mRequest.mutable_urdf()->robot_name();
  m_mNameList.insert(pair<string, string>(mRequest.proxy_name(),m_sLastJoinRobotName));

  // 4. add URDF file of new join client in our urdf map
  URDFMsg* newURDF = mRequest.mutable_urdf();
  m_mURDF.insert(pair<string, URDFMsg* >(newURDF->robot_name(), newURDF));

  cout<<"[StateKeeper/RegisterLocalSim] proxy '"<<mRequest.proxy_name()<<"' join statekeeper success. Save its urdf file with robot name '"
     <<newURDF->robot_name()<<"' success. Now Statekeeper has "<<m_mURDF.size()<<" URDF files."<<endl;

}

////////////////////////////////////////////////////////////////////////

// if a new robot add to statekeeper, statekeeper will send its URDF and state to all other clients.

bool StateKeeper::CheckIfNeedToSendProxysURDF()
{
  if(m_bSendURDFtoProxys == true)
  {
    // prepare URDF file for send
    std::map<string,URDFMsg*>::iterator iter = m_mURDF.find(m_sLastJoinRobotName);
    if(iter == m_mURDF.end())
    {
      cout<<"[StateKeeper/CheckIfNeedToSendProxysURDF] Error! Cannot find "<<m_sLastJoinRobotName<<" in URDF map."<<endl;
      return false;
    }
    URDFMsg* pURDF = iter->second;


    // call rpc method of all other client, one by one.
    vector<string> vProxyNames= m_Node.GetSubscribeClientName();
    for (unsigned int i=0;i!=vProxyNames.size();i++)
    {
      string sProxyName = vProxyNames[i];
      if(sProxyName!= GetRobotLastName( m_sLastJoinRobotName))
      {
        LocalSimAddNewRobotReqMsg mRequest;

        // 1. set robot name
        mRequest.set_robot_name(pURDF->robot_name());

        // 2. set URDF
        mRequest.mutable_urdf()->set_robot_name(pURDF->robot_name());
        mRequest.mutable_urdf()->set_xml(pURDF->xml());
        //  mRequest.mutable_urdf()->set_mesh(mURDF.mesh());  // ******implement later. Not support yet.*******

        // 3. set pose for body base
        mRequest.mutable_init_pose()->set_x(m_eLastJoinRobotInitPose[0]);
        mRequest.mutable_init_pose()->set_y(m_eLastJoinRobotInitPose[1]);
        mRequest.mutable_init_pose()->set_z(m_eLastJoinRobotInitPose[2]);
        mRequest.mutable_init_pose()->set_p(m_eLastJoinRobotInitPose[3]);
        mRequest.mutable_init_pose()->set_q(m_eLastJoinRobotInitPose[4]);
        mRequest.mutable_init_pose()->set_r(m_eLastJoinRobotInitPose[5]);

        // 4. call rpc method
        LocalSimAddNewRobotRepMsg mReply;
        string sServiceName = vProxyNames[i]+"/AddRobotByURDF";
        if( m_Node.call_rpc(sServiceName, mRequest, mReply) ==false)
        {
          cout<<"[StateKeeper/SendURDFToProxys] Error Call rpc method of "<<sServiceName<<endl;
          return false;
        }
        else
        {
          cout<<"[StateKeeper/SendURDFToProxys] Send URDF file with robot name '"<<pURDF->robot_name()<<"' to '"<<sServiceName<<"' Success."<<endl;
        }
      }
    }
    m_bSendURDFtoProxys = false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////

// if proxy A exit StateKeeper, call rpc method of all other proxys to delete robot in Proxy A.

bool StateKeeper::CheckIfNeedToDeleteRobotInAllProxys()
{
  // check if there is robot just exit
  if(m_Node.GetSubscribeClientName().size() >= m_mNameList.size())
  {
    return true;
  }

  // prepare name of proxy that need to delete a robot
  vector<string> vSubscribeClientName = m_Node.GetSubscribeClientName();

  std::map<string,string>::iterator iter = m_mNameList.begin();
  string sNameofProxyJustExit;
  for(;iter!=m_mNameList.end();iter++)
  {
    sNameofProxyJustExit = iter->first;
    bool bflag = false;
    for(unsigned int i=0;i!=vSubscribeClientName.size();i++)
    {
      if(vSubscribeClientName[i] == sNameofProxyJustExit)
      {
        bflag = true;
        break;
      }
    }

    // if we cannot find element in NameList map of current subscribe client name list, then this proxy is the one just exit StateKeeper
    if(bflag==false)
    {
      string sRobotNeedToBeDeleteName = iter->second;
      cout<<"[StateKeeper] Detect robot: "<<sRobotNeedToBeDeleteName<<" need to be delete!"<<endl;

      LocalSimDeleteRobotReqMsg mRequest;
      mRequest.set_robot_name(sRobotNeedToBeDeleteName);
      LocalSimDeleteRobotRepMsg  mReply;

      // call rpc method of all subscribe client
      for(unsigned int j=0;j!=m_Node.GetSubscribeClientName().size();j++)
      {
        string sProxyName = m_Node.GetSubscribeClientName()[j];
        string sServiceName = sProxyName+"/DeleteRobot";
        if(m_Node.call_rpc(sServiceName, mRequest, mReply)== false)
        {
          cout<<"[StateKeeper] Fatal error! cannot call rpc method "<<sServiceName<<" to delete "<<sRobotNeedToBeDeleteName<<endl;
        }
        else if(mReply.message()=="DeleteRobotSuccess")
        {
          cout<<"[StateKeeper] Call "<<sServiceName<<" to delete "<<sRobotNeedToBeDeleteName<<" success."<<endl;
        }
      }
    }
  }

  // delete proxy from name list
  m_mNameList.erase(sNameofProxyJustExit);

  return true;
}

////////////////////////////////////////////////////////////////////////

// update world state by collecting all robots' pose and command.

bool StateKeeper::ReceiveWorldFullState()
{
  // create a new world state table to store new world state.
  WorldFullStateMsg mWorldFullState;
  mWorldFullState.set_time_step(m_iTimeStep);

  // get state of robot one by one
  vector<string> RobotsNames= m_Node.GetSubscribeClientName();

  // here we use max try to avoid infinia wait
  int iMaxTry=100;
  while (RobotsNames.size()!=0)
  {
    for (unsigned int i= 0; i!=RobotsNames.size();)
    {
      string sServiceName=RobotsNames[i]+"/RobotState";
      RobotFullStateMsg mRobotFullState;

      // save world state information
      if(m_Node.receive(sServiceName, mRobotFullState )==true && mRobotFullState.time_step()==m_iTimeStep+1)
      {
        // set pose info
        mWorldFullState.add_robot_state()->CopyFrom(mRobotFullState);

        vector<string>::iterator it=RobotsNames.begin()+i;
        RobotsNames.erase(it);
        i=0;
      }
      else if(iMaxTry==0)
      {
        cout<<"[StateKeeper/WorldState] ERROR!!! Cannot Get State For "<<sServiceName<<". Break!"<<endl;
        return false;
      }
      else
      {
        i++;
        iMaxTry--;
      }
    }
  }

  // update world state
  m_WorldFullState=mWorldFullState;
  cout<<"[StateKeeper/WorldState] Get World State success. New world state size: "<<m_WorldFullState.robot_state_size()<<endl;
  return true;
}

////////////////////////////////////////////////////////////////////////

// publish world state

bool StateKeeper::PublishWorldFullState()
{
  // update time step once we gather all robot state
  m_iTimeStep++;

  m_WorldFullState.set_time_step(m_iTimeStep);

  bool bStatus=false;
  while(bStatus==false)
  {
    bStatus=m_Node.publish("WorldState",m_WorldFullState);
    if( bStatus==false)
    {
      printf("[World State] ERROR: publishing RobotState fail. Try again\n" );
    }
  }
  cout<<"[StateKeeper/WorldState] Publish world state success. size is "<<m_WorldFullState.robot_state_size()<<" time step is "<<m_WorldFullState.time_step()<<endl;
  return true;
}

////////////////////////////////////////////////////////////////////////

// once all player exit StateKeeper, we need to clear relative information.

void StateKeeper::ClearAllPreviousMessageIfNecessary()
{
  m_sLastJoinRobotName.clear();
  m_bSendURDFtoProxys = false;
  m_mURDF.clear();
  m_WorldFullState.Clear();
  m_mNameList.clear();
}

///////////////////////////////////////////////////////////////////
/////
///// MAIN LOOP:
///// Creates a StateKeeper object to start syncing all of our RobotProxies.
///// fo sho.
/////
///////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  // parse command line arguments
  GetPot cl( argc, argv );
  string sStateKeeperName = cl.follow("StateKeeper",1,"-n");
  std::string sWorldURDF = cl.follow( "/Users/malu/Code/Luma/Sim-HAL/urdf/World.xml", 1, "-w" );

  if( argc != 5 && argc !=1 ){
    std::cout<<"USAGE: ./StateKeeper -n <StateKeeperName>"
            <<" -w <WorldURDFFilePath>"<<std::endl;
    return -1;
  }

  StateKeeper app(sStateKeeperName, sWorldURDF);
  app.InitRobotPose();

  // main loop. The main job of statekeeper is to collect and publish world state
  while(1)
  {
    if(app.m_Node.GetSubscribeClientName().size()==0)
    {
      cout<<"No subscribe client in server. Waiting.."<<endl;
      app.ClearAllPreviousMessageIfNecessary(); // bad form, need to improve in the future
      app.InitRobotPose();                      // bad form, need to improve in the future
      sleep(2);
    }
    else
    {
      //            cout<<"*********************** in time step "<<app.m_iTimeStep<<" **************************"<<endl;
      app.CheckIfNeedToSendProxysURDF();
      app.CheckIfNeedToDeleteRobotInAllProxys();

      app.ReceiveWorldFullState();
      app.PublishWorldFullState();
    }
  }
  return 0;
}
