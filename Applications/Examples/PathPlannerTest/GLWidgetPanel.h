#ifndef GLWIDGETPANEL_H
#define GLWIDGETPANEL_H

#include <Mvlpp/SE3.h>
#include "RigidBody.h"

#include "pangolin/pangolin.h"
#include "SceneGraph/SceneGraph.h"
#include "Widgets/GLWidgetView.h"
#include "Widgets/nvGlutWidgets.h"
#include "Widgets/nvGLWidgets.h"
#include "Widgets/nvShaderUtils.h"
#include "Widgets/nvWidgets.h"

/*
  float& fScale =
  CVarUtils::CreateCVar("car.Scale", 0.1f, "rendering scale option");
  float& fRoll =
  CVarUtils::CreateCVar("car.Roll", 0.0f, "rendering scale option");
  float& fPitch =
  CVarUtils::CreateCVar("car.Pitch", 0.0f, "rendering scale option");
  float& fYaw =
  CVarUtils::CreateCVar("car.Yaw", 0.0f, "rendering scale option");
*/

using namespace SceneGraph;
using namespace pangolin;

class GLWidgetPanel : public GLObject {
 public:

  GLWidgetPanel(){
  }

  void Init(int w, int h){
    m_Ui.init(w, h);
  }

  virtual void DrawUI() = 0;

  void DrawCanonicalObject(){
    DrawUI();
  }

  int DoNumericLineEdit(nv::Rect& rect, const char *label, int* nNumber){
    m_Ui.doLabel(rect,label);
    int charsReturned;
    char sNumber[10];
    sprintf(sNumber,"%d",*nNumber);
    m_Ui.doLineEdit(rect,sNumber,10,&charsReturned);
    *nNumber = atoi(sNumber);
    return charsReturned;
  }

  template<typename T>
  T GetVar(std::string name){
    return (T)m_mVars[name];
  }

  GLWidgetPanel& SetVar(std::string name, void *ptr){
    m_mVars[name] = ptr;
    return *this;
  }

  nv::GlutUIContext* GetUIContext(){ return &m_Ui; }

 protected:
  nv::Rect m_Rect;
  nv::GlutUIContext m_Ui;
  std::map<std::string,void *> m_mVars;

};

#endif // GLWIDGETPANEL_H
