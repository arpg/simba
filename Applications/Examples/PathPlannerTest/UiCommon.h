#ifndef UICOMMON_H
#define UICOMMON_H

#include "pangolin/pangolin.h"
#include "SceneGraph/SceneGraph.h"
#include "GLWidgetPanel.h"
#include "CVars/CVar.h"
#include "CVarHelpers.h"

struct PlannerHandler : public SceneGraph::HandlerSceneGraph
{

  PlannerHandler(SceneGraph::GLSceneGraph& graph,
                 pangolin::OpenGlRenderState& cam_state,
                 pangolin::AxisDirection enforce_up=pangolin::AxisNone,
                 float trans_scale=0.01f,
                 std::vector<GLWidgetPanel*>* pPanels = NULL) :
      HandlerSceneGraph(graph,cam_state,enforce_up,trans_scale){
    m_pWidgetPanels = pPanels;
  }

  void Mouse(pangolin::View& view, pangolin::MouseButton button,
             int x, int y, bool pressed, int button_state){
    if( m_pWidgetPanels != NULL) {
      //convert pangolin button to GLUT button
      int glutButton = (int)button;
      int log2 = 0;
      while (glutButton >>= 1) ++log2;
      for(size_t ii = 0; ii < m_pWidgetPanels->size() ; ii++){
        m_pWidgetPanels->at(ii)->GetUIContext()->mouse(log2,
                                                       (int)!pressed,
                                                       x,view.v.h - y);
      }
    }
    HandlerSceneGraph::Mouse(view,button,x,y,pressed,button_state);
  }

  void MouseMotion(pangolin::View& view, int x, int y, int button_state){
    if( m_pWidgetPanels != NULL) {
      for(size_t ii = 0; ii < m_pWidgetPanels->size() ; ii++){
        m_pWidgetPanels->at(ii)->GetUIContext()->mouseMotion(x,view.v.h - y);
      }
    }
    HandlerSceneGraph::MouseMotion(view,x,y,button_state);
  }

  void Keyboard(View& view, unsigned char key, int x, int y, bool pressed){
    if( m_pWidgetPanels != NULL && pressed) {
      for(size_t ii = 0; ii < m_pWidgetPanels->size() ; ii++){
        m_pWidgetPanels->at(ii)->GetUIContext()->keyboard(key,x,y);
      }
    }
    HandlerSceneGraph::Keyboard(view,key,x,y,pressed);
  }

  std::vector<GLWidgetPanel*>* m_pWidgetPanels;

};

struct ActivateScissorBlendedDrawFunctor
{

  ActivateScissorBlendedDrawFunctor(
      SceneGraph::GLObject& glObject,
      const pangolin::OpenGlRenderState& renderState)
      :glObject(glObject), renderState(renderState){
  }

  void operator()(pangolin::View& view) {
    view.ActivateAndScissor(renderState);
    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0,1.0,1.0,1.0);
    glBegin(GL_QUADS);                      // Draw A Quad
    glVertex3f(-1.0f, 1.0f, 0.0f);              // Top Left
    glVertex3f( 1.0f, 1.0f, 0.0f);              // Top Right
    glVertex3f( 1.0f,-1.0f, 0.0f);              // Bottom Right
    glVertex3f(-1.0f,-1.0f, 0.0f);              // Bottom Left
    glEnd();
    glPopAttrib();
    glObject.DrawObjectAndChildren(eRenderVisible);
  }

  SceneGraph::GLObject& glObject;
  const pangolin::OpenGlRenderState& renderState;

};

struct Waypoint
{
  Waypoint(): m_eType(eWaypoint_Normal){}

  SceneGraph::GLWayPoint m_Waypoint;
  WaypointType m_eType;
};

// // Writing waypoints to file

// inline std::ostream& operator<<( std::ostream& Stream, Waypoint& waypoint ){
//   //write the waypoint to file
//   Eigen::MatrixXd pose = waypoint.m_Waypoint.GetPose();
//   Stream << pose << "," << (int)waypoint.m_eType;
//   return Stream;
// }

// inline std::istream& operator>>( std::istream& Stream, Waypoint& waypoint ){
//   //read the waypoiint from file
//   Eigen::MatrixXd pose;
//   Stream >> pose;
//   Eigen::Vector6d vecPose = pose;
//   waypoint.m_Waypoint.SetPose(vecPose);

//   //now read the type
//   char str[256];
//   Stream.getline(str,255);
//   waypoint.m_eType = (WaypointType)std::strtol(str,NULL,10);
// }

#endif // UICOMMON_H
