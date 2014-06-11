#ifndef PLANNERGUI_H
#define PLANNERGUI_H

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

#define UI_PANEL_WIDTH 200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT

#include <stdio.h>
#include "CarPlannerCommon.h"
#include "BulletCarModel.h"
#include "boost/thread.hpp"
#include "ClickHandler.h"
#include "pangolin/pangolin.h"
#include "SceneGraph/SceneGraph.h"
#include "GLCar.h"
#include "GLWidgetPanel.h"
#include "UiCommon.h"


class PlannerGui
{
 public:
  enum PlannerViewType{
    eNeutral,
    eFollowNear,
    eFollowFar,
    eFollowWheel
  };

  enum PlannerGuiCommands{
    eChangeView,
    eVideoToggle,
    eIncreaseWpVel,
    eDecreaseWpVel,
  };

  enum StatusLineLocation {
    eTopLeft = 0,
    eBottomLeft = 1
  };

  struct StatusLine
  {
    StatusLineLocation m_Location;
    SceneGraph::GLText m_GLText;
    Eigen::Vector2d m_dPos2D;
  };

  struct Car
  {
    SceneGraph::GLLineStrip m_CarLineSegments;
    GLCar m_GLCar;
    GLAxis m_Axis;
  };

  PlannerGui();
  ~PlannerGui();

  /// Render a frame
  void Render();

  /// Initializes the GUI.
  /// The GLObject for the terrain. This will get added to the
  /// Scenegraph and rendered
  void Init(SceneGraph::GLObject* pTerrain);
  void Init(const std::string sTerrainMeshFileName, GLMesh *pMesh,
            const bool bViconCoords = false);

  /// Adds a new GLObject to the scenegraph
  void AddGLObject(GLObject* pObject, bool bCastShadows = false) {
    m_SceneGraph.AddChild(pObject);
    if(bCastShadows){
      m_pLight->AddShadowCaster(pObject);
    }
  }
  //Removes a GLObject
  void RemoveGLObject(GLObject* pObject){ m_SceneGraph.RemoveChild(pObject); }
  /// Adds a new 2D object to the scenegraph
  void Add2DGLObject(GLObject* pObject) { m_SceneGraph2d.AddChild(pObject); }
  /// Adds a GLPanel object to the 2D scenegraph and hooks up its events
  void AddPanel(GLWidgetPanel* pPanel);

  /// Adds a new status line to the gui,
  /// to allow the user to add text, Returns the ID of this status line
  int AddStatusLine(StatusLineLocation location);
  /// Sets the text on a previously created status
  void SetStatusLineText(int id, std::string text);

  /// Add a new car to the scene. Returns the ID of the car
  int AddCar(Eigen::Vector3d vBodyDim, Eigen::Vector3d vWheelDim);
  int AddCar(double wheelbase, double width, double height,
             double wheel_radius, double wheel_width);
  /// Sets the position and rotation of the car and the wheels
  /// used the vehiclestate structure passed.
  /// The structure containing the state of the car and wheels
  /// If bAddToTrajectory is true, a new point will be added to the
  /// linestrip that tracks the car trajectory
  void SetCarState(const int& id, const VehicleState& state,
                   bool bAddToTrajectory = false);
  /// Sets a car to follow, or if given an id of -1, will disable following
  void SetFollowCar(const int& id) {
    m_pFollowCar = id == -1 ? NULL : m_vCars[id];
  }
  /// Sets the visibility flag of the chosen car
  void SetCarVisibility(const int& id, const bool& bVisible);
  bool GetCarVisibility(const int& id) {
    return m_vCars[id]->m_GLCar.IsVisible();
  }
  /// Sets the visibility of the grid
  void SetGridVisibility(const bool bVisible) { m_Grid.SetVisible(bVisible ); }
  bool GetGridVisibility() { return m_Grid.IsVisible(); }

  /// Returns the current number of added waypoints
  int WaypointCount() { return m_vWaypoints.size(); }
  /// Adds a new waypoint, returns the ID
  int AddWaypoint(const Eigen::Vector6d& pose, const double& velocity);
  int AddWaypoint(const Eigen::Matrix4d& T_po, const double &velocity);
  int AddWaypoint(const VehicleState &state);
  /// Returns a pointer to the waypoint object
  Waypoint* GetWaypoint(const int& id) { return m_vWaypoints[id]; }
  /// Deletes all waypoints
  void ClearWaypoints();
  /// Clears the car trajectory
  void ClearCarTrajectory(const int& id) {
    m_vCars[id]->m_CarLineSegments.ClearLines();
  }
  /// Sets the dirty flag on all waypoints to true
  void SetWaypointDirtyFlag(bool bFlag);
  std::vector<double> GetClickPose(){ return m_Handler->WPose; }
  std::vector<double> GetClickNorm(){ return m_Handler->Norm; }

 private:
  /// Populates the initial scenegraph items
  void _PopulateSceneGraph();

  /// Function which takes in keyboard commands and executes
  /// the corresponding action
  void _CommandHandler(const PlannerGuiCommands& command);

  /// Sets the camera as specified by the given view type
  void _SetViewType(const PlannerViewType& eViewType);

  //objects
  SceneGraph::GLObject* m_pTerrain;

  //view related variables
  SceneGraph::GLShadowLight *m_pLight;
  SceneGraph::GLShadowLight *m_pStaticLight;
  SceneGraph::GLSceneGraph m_SceneGraph;
  SceneGraph::GLSceneGraph m_SceneGraphWidgets;
  SceneGraph::GLSceneGraph m_SceneGraph2d;
  pangolin::OpenGlRenderState m_RenderState;
  pangolin::OpenGlRenderState m_RenderState2d;
  pangolin::OpenGlRenderState m_RenderStateWidget;
  pangolin::View* m_pView;
  pangolin::View* m_pPanelView;
  ClickHandler*   m_Handler;

  //vector of text objects
  std::vector<Waypoint*> m_vWaypoints;

  std::vector<StatusLine*> m_vStatusLines;
  std::vector<Car*> m_vCars;

  //vector of widgets panels
  std::vector<GLWidgetPanel*> m_vWidgetPanels;

  Car* m_pFollowCar;
  //boost::mutex m_DrawMutex;
  PlannerViewType m_eViewType;
  int m_nSelectedWaypoint;
  GLGrid m_Grid;
};

#endif // PLANNERGUI_H
