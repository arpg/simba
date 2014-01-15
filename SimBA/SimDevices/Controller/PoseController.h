#ifndef POSECONTROLLER_H
#define POSECONTROLLER_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pangolin/pangolin.h>                // for open GL state management

using namespace std;

// the following is a naive pose controller that read pose from data file for robot or camera to apply
class PoseController
{
public:
    void Init(string sDataFile)
    {
        m_bInit = false;
        if(sDataFile == "None")
        {
            cout<<"[Device/PoseController] Did Not use Pose Controlloer"<<endl;
        }
        else
        {
            m_bInit = true;
            std::ifstream infile(sDataFile);
            cout<<"[Device/PoseController] File Path is "<<sDataFile<<endl;

            std::string line;
            int         PoseNum =0;
            while (std::getline(infile, line))
            {
                std::istringstream iss(line);
                double x,y,z,p,q,r;
                if (!(iss >> x >> y >> z >> p >>q >>r ))
                {
                    cout<<"[Device/PoseController] Finish Reading Pose Information. Read "<<PoseNum<<" Pose in Total."<<endl;
                    break;
                }
                else
                {
                    PoseNum++;
                    Eigen::Vector6d Pose;
                    Pose<<x,y,z,p,q,r;
                    m_vPoseVector.push_back(Pose);
                }
            }

            m_CurPoseIndex = 0;
        }
    }



    Eigen::Vector6d ReadNextPose()
    {
        Eigen::Vector6d Pose;

        if(m_bInit == false)
        {
            Pose<<1,88,99,111,00,44;
            return Pose;
        }
        else
        {
            if(m_CurPoseIndex !=m_vPoseVector.size())
            {
                Pose = m_vPoseVector[m_CurPoseIndex];
                m_CurPoseIndex++;
            }
            else
            {
                cout<<"[Device/PoseContorller] Finish read pose of a loop. Now restart."<<endl;
                m_CurPoseIndex = 0;
                Pose = m_vPoseVector[m_CurPoseIndex];
                m_CurPoseIndex++;
            }

        }
        return Pose;

    }

    // =======================================================
    int                          m_CurPoseIndex;
    vector<Eigen::Vector6d>      m_vPoseVector;
    bool                         m_bInit;
};

#endif // POSECONTROLLER_H
