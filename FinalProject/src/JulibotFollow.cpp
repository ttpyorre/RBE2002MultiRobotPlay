#include "JulibotFollow.h"
#include "ir_codes.h"
OpenMV camera;

#define SPEEDMOD 10
#define APRIL_TAG_SIZE 4.5
#define CAMERA_X_AXLE_LENGTH 160
#define WIDTH_25CM_FROM_CAMERA 30
#define LENGTH_FROM_CAMERA 50

uint8_t JulibotFollow::FindAprilTags()
{
  uint8_t tagCount = camera.getTagCount();
  if(tagCount) 
  {
    AprilTagDatum tag;
    if(camera.readTag(tag) && millis() - lastHandleUpdateJulibot >= 50)
    {
      lastHandleUpdateJulibot = millis();

      //X, and Y coordinated of tag.
      cx = tag.cx;
      cy = tag.cy;

      //Width and Height of tag
      width = tag.w;
      height = tag.h;

      rot = tag.rot;
    }
  }

  return tagCount;
}

double JulibotFollow::getJulibotDistance()
{
  //Getting distance for Julibot.
  double focalLength = (WIDTH_25CM_FROM_CAMERA * LENGTH_FROM_CAMERA) / APRIL_TAG_SIZE;
  double distance = (APRIL_TAG_SIZE * focalLength) / width;
  
  return distance;
}


void JulibotFollow::tagStandoff()
{
  uint8_t tagCount = camera.getTagCount();

  if(tagCount > 0)
  {
    //Centers for the distance with the camera and the apriltag.
    float errorX = cx - (CAMERA_X_AXLE_LENGTH / 2);
    float effortX = piStandoff.computeEffort(errorX);

    double distance =  getJulibotDistance();

    //gives effort to wheels to turn the romi to stay a wanted distance from apriltag.
    float errorTurn = LENGTH_FROM_CAMERA - distance;
    float effortTurn = piStandoffTurn.computeEffort(errorTurn);
    
    //Finally we give the efforts for the wheels.
    leftEffort = -effortX - effortTurn/6;
    rightEffort = -effortX + effortTurn/6;
  } 
}