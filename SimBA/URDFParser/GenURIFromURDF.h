#ifndef GENURIFROMURDF_H
#define GENURIFROMURDF_H

#include <stdlib.h>
#include <string>
#include <URDFParser/TinyXMLTool.h>

using namespace std;

const char* GenURIFromURDF(string sDeviceName, string sURDFPath)
{
    XMLDocument doc;
    if(doc.LoadFile(sURDFPath.c_str()) !=0)
    {
         printf("[GenURIFromURDF] Fatal Error! Cannot open %s\n", sURDFPath.c_str());
         exit(-1);
    }
    else
    {
        cout<<"Open file: "<<sURDFPath<<" success."<<endl;
    }

    XMLElement *pParent=doc.RootElement();
    XMLElement *pElement=pParent->FirstChildElement();

    string sURI; // final URI we need
    bool bInitDeviceFlag = false; // mark if we init device successfully

    // read high level parent (root parent)
    while (pElement)
    {
        const char* sRootContent = pElement->Name();

        // create sim Sensor device
        if(strcmp(sRootContent,"Sensor")==0)
        {
            string sType = GetAttribute( pElement, "Type");

            // ============== for Camera Device ===============
            if(sType == "Camera")
            {
                string sName(GetAttribute(pElement,"Name") );

                // find the sensor we want to init
                if(sName == sDeviceName)
                {
                    bInitDeviceFlag = true;

                    string sDriver = GetAttribute( pElement, "Driver");

                    //------------------------------------------------ Kinect
                    if(sDriver == "Kinect" )
                    {
                        sURI = sURI + "openni:[";
                        string sParam;
                        string sSize  = GetAttribute(pElement,"Size");
                        string sMode  = GetAttribute(pElement,"Mode");
                        string sFPS   = GetAttribute(pElement,"FPS");
                        string sAlign = GetAttribute(pElement,"Align");

                        //----------------------------------------- set size
                        sParam = sParam + "size="+sSize+",";

                        //----------------------------------------- set mode
                        if(sMode=="RGBD")
                        {
                            sParam = sParam +"rgb=1,depth=1,";
                        }
                        else if(sMode == "RGB")
                        {
                            sParam = sParam +"rgb=1,depth=0,";
                        }
                        else if(sMode == "Depth")
                        {
                            sParam = sParam +"rgb=0,depth=1";
                        }

                        //----------------------------------------- set FPS
                        sParam = sParam + "FPS="+sFPS+",";

                        //----------------------------------------- set align
                        if(sAlign=="true")
                        {
                            sParam = sParam + "align=1]";
                        }
                        else
                        {
                            sParam = sParam + "align=0]";
                        }

                        sURI = sURI + sParam +"//";
                    }

                    //------------------------------------------------ Node2Cam
                    else if(sDriver == "Node2Cam")
                    {
                        string sLocation(pElement->Attribute("Location"));
                        if((signed)sLocation.find("@")!=-1)
                        {
                            cout<<"[RPG/ParseURDF] Detect a node cam! Name is '"<<sName<<endl;
                            sURI = sURI + "node2cam:[";
                            string sParam;
                            sParam = "device=" + sDeviceName+",";
                            sParam =sParam+ "host=" + sLocation.substr(1,sLocation.size()-1) + "]";
                            sURI = sURI +sParam +"//";
                        }
                        else
                        {
                            cout<<"Error! You try to init Node2Cam but did not set Host Name! Please check your URDF file!"<<endl;
                            exit(-1);
                        }
                    }

                }

            }
            else if(sType == "Controller")
            {

            }

        }
        // read next parent element
        pElement=pElement->NextSiblingElement();
    }

    if(bInitDeviceFlag == false)
    {
       cout<<"[GenURUFormURDF] Cannot Find "<<sDeviceName<<" in Robot URDF file. Please make sure you enter the right device name!"<<endl;
       exit(-1);
    }

    cout<<"uri is "<<sURI<<endl;
    return sURI.c_str();

}

#endif // GENURIFROMURDF_H
