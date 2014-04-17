#ifndef GLHEIGHTMAP_H
#define GLHEIGHTMAP_H

#include "SceneGraph/GLObject.h"

namespace SceneGraph {

/// The SceneGraph heightmap
class GLHeightmap : public GLObject
{
public:

  GLHeightmap(double* x_array, double* y_array, double* z_array,
              double row_count, double col_count){
    X_ = x_array;
    Y_ = y_array;
    Z_ = z_array;
    row_count_ = row_count;
    col_count_ = col_count;
    render_ = false;
  }

  //////////////////////////////////////////////////////////////////////////////
  void DrawCanonicalObject(){
    if(render_){
      glBegin(GL_TRIANGLES); // Render Polygons
    } else {
      glBegin(GL_LINES); // Render Lines Instead
    }
    int index = 0;
    for (int i=0;i<row_count_-1;i++) {
      // Right now, I can't get the max height, so I color
      // the mesh lighter as we go down the rows.
      float color_change = i/row_count_;
      glColor3f(1,color_change,1);
      for (int j=0;j<col_count_-1;j++) {
        // Triangle 1
        index = j*row_count_+i;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        index = j*row_count_+i+1;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        index = (j+1)*row_count_+i+1;
        glVertex3f( X_[index], Y_[index], Z_[index] );

        // Triangle 2
        index = j*row_count_+i;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        index = (j+1)*row_count_+i+1;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        index = (j+1)*row_count_+i;
        glVertex3f( X_[index], Y_[index], Z_[index] );
      }
    }
    glEnd();
    // Reset color
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  }

private:
  double* X_;
  double* Y_;
  double* Z_;
  double row_count_;
  double col_count_;
  bool render_;
};

}

#endif // GLHEIGHTMAP_H
