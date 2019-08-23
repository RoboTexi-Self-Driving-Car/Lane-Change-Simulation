#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "vec2D.h"

static float manhattanDistance(const Vector2f& v1, const Vector2f& v2) {
  float distance = abs(v1[0]-v2[0]) + abs(v1[1] - v2[1]);
  return distance;
}

struct Line {
  int x1,y1,x2,y2;
  Line(float _x1, float _y1, float _x2, float _y2):x1(_x1),y1(_y1),x2(_x2),y2(_y2) {}
  Line(vector<int>& row) {
    x1 = row[0] * (Globals::constant.BLOCK_TILE_SIZE);
    y1 = row[1] * (Globals::constant.BLOCK_TILE_SIZE);
    x2 = row[2] * (Globals::constant.BLOCK_TILE_SIZE);
    y2 = row[3] * (Globals::constant.BLOCK_TILE_SIZE);
  }
  Vector2f getstart() { return Vector2f(x1, y1);}
  Vector2f getend() {return Vector2f(x2, y2);}
};

class Block {
private:
  int startx,starty;
  int endx, endy;
  float centerX, centerY;

public:
  Block(): startx(0), starty(0), endx(0),endy(0){}

  Block(vector<int>& blockdata) {
    assert(blockdata.size() == 4);
    int unit = Globals::constant.BLOCK_TILE_SIZE;
    startx = blockdata[0]*unit;
    starty = blockdata[1]*unit;
    endx = blockdata[2]*unit;
    endy = blockdata[3]*unit;
    centerX = (startx + endx)/2.0;
    centerY = (starty + endy)/2.0;
  }

  Vector2f getCenter() const { return Vector2f(centerX, centerY); }

  int getWidth() const { return abs(endx - startx); }

  int getHeight()  const{ return abs(endy - starty); }

  bool containsPoint(int x, int y) const {
    if (x < startx) return false;
    if (y < starty) return false;
    if (x > endx) return false;
    if (y > endy) return false;
    return true;
  }

  //larger one
  bool containsPointLarger(int x, int y) const {
    int size = 2;
    int startx1 = startx - size;
    int starty1 = starty - size;
    int endx1 = endx + size;
    int endy1 = endy + size;
    if (x < startx1) return false;
    if (y < starty1) return false;
    if (x > endx1) return false;
    if (y > endy1) return false;
    return true;
  }
};

#endif // GEOMETR_H
