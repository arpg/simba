#ifndef GLHEIGHTMAP_H
#define GLHEIGHTMAP_H

#include "SceneGraph/GLObject.h"
#include "SceneGraph/GLLineStrip.h"

namespace SceneGraph {

/// The SceneGraph heightmap
class GLHeightMap : public GLObject
{
public:

  GLHeightMap(double* x_array, double* y_array, double* z_array,
              double row_count, double col_count){
    owns_texture_ = false;
    texture_id_ = 0;
    X_ = x_array;
    Y_ = y_array;
    Z_ = z_array;
    row_count_ = row_count;
    col_count_ = col_count;
    render_ = false;
  }

  ///////////////////////////////////////////////////////////////////////////////

  void ClearTexture() {
    if (texture_id_ > 0) {
      if (owns_texture_) {
        glDeleteTextures(1,&texture_id_);
      }
      texture_id_ = 0;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////

  void SetCheckerboard() {
    ClearTexture();
    // Texture Map Init
    GLubyte img[TEX_W][TEX_H][3]; // after glTexImage2D(), array is no longer needed
    for (int x=0; x<TEX_W; x++) {
      for (int y=0; y<TEX_H; y++) {
        GLubyte c = ((x&16)^(y&16)) ? 255 : 0; // checkerboard
        img[x][y][0] = c;
        img[x][y][1] = c;
        img[x][y][2] = c;
      }
    }
    // Generate and bind the texture
    owns_texture_ = true;
    glGenTextures( 1, &texture_id_ );
    glBindTexture( GL_TEXTURE_2D, texture_id_ );
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D( GL_TEXTURE_2D, 0,
                  GL_RGB, TEX_W, TEX_H, 0, GL_RGB, GL_UNSIGNED_BYTE, &img[0][0][0] );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  }

  ///////////////////////////////////////////////////////////////////////////////

  void DrawCanonicalObject(){
    if(render_){
      glBegin(GL_TRIANGLES); // Render Polygons
    } else {
      glBegin(GL_LINES); // Render Lines Instead
    }
    glColor3f(1,0,1);
    int index = 0;
    for (int i=0;i<row_count_-1;i++) {
      for (int j=0;j<col_count_-1;j++) {
        // Triangle 1
        index = j*row_count_+i;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        glTexCoord2f(0.0, 0.0);
        index = j*row_count_+i+1;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        glTexCoord2f(0.0, 1.0);
        index = (j+1)*row_count_+i+1;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        glTexCoord2f(1.0, 1.0);

        // Triangle 2
        index = j*row_count_+i;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        glTexCoord2f(0.0, 0.0);
        index = (j+1)*row_count_+i+1;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        glTexCoord2f(1.0, 1.0);
        index = (j+1)*row_count_+i;
        glVertex3f( X_[index], Y_[index], Z_[index] );
        glTexCoord2f(1.0, 0.0);
      }
    }
    glEnd();

    if(texture_id_) {
      glDisable( GL_TEXTURE_2D );
    }
  }


protected:
 const static int TEX_W = 64;
 const static int TEX_H = 64;
 bool   owns_texture_;
 GLuint texture_id_;

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
