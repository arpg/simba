
/// STATEKEEPER FUNCTIONS
bool RegisterRobot(RobotsManager* pRobotsManager);
bool RegisterWithStateKeeper();
void AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                    LocalSimAddNewRobotRepMsg& mReply);
void DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                 LocalSimDeleteRobotRepMsg& mReply);
bool PublishRobotToStateKeeper();
bool ReceiveWorldFromStateKeeper();
