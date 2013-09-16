#ifndef CONVERTNAME_H
#define CONVERTNAME_H

#include <Network/Messages.h>
#include <Network/WorldState.h>


// Get first name from robot name. robot name e.g. robot1@proxy1
string GetRobotFirstName(string sRobotFullName)
{
    string sFirstName;
    int Index = sRobotFullName.find("@");
    sFirstName = sRobotFullName.substr(0,Index);
    return sFirstName;
}


// Get last name from robot name.
string GetRobotLastName(string sRobotFullName)
{
    string sLastName;
    int Index = sRobotFullName.find("@");
    sLastName = sRobotFullName.substr(Index+1,sRobotFullName.size());
    return sLastName;
}



// Get first name from full name. format of Full name is FirstName@MiddleName@LastName. e.g. RCamera@Robot1@Proxy1
string GetFirstName(string sFullName)
{
    string sFirstName;
    int Index = sFullName.find("@");
    if(Index == -1)
    {
        sFirstName = sFullName;
    }
    else
    {
        sFirstName = sFullName.substr(0, Index);
    }

    return sFirstName;
}


// Get last from full name. format of Full name is FirstName@MiddleName@LastName. e.g. RCamera@Robot1@Proxy1
string GetLastName(string sFullName)
{
    string sLastName;
    int Index = sFullName.find_last_of("@");
    sLastName = sFullName.substr(Index+1,sFullName.size()-Index-1);
    return sLastName;
}


// Get robot name (middle and last name) from full name
string GetRobotNameFromFullName(string sFullName)
{
    string sRobotName;
    int Index = sFullName.find("@");
    sRobotName = sFullName.substr(Index+1, sFullName.size()-Index);
    return sRobotName;
}


#endif // CONVERTNAME_H
