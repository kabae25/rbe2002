#include <Arduino.h>
#include <Wire.h>
#include <openmv.h>
#include <Vision.h>

OpenMV camera;

void Vision::init(void)
{
  Wire.begin();
  Wire.setClock(100000ul);
}

uint8_t Vision::FindAprilTags(AprilTagData& data)
{
    uint8_t tagCount = camera.getTagCount();
    if(tagCount) 
    {
      //Serial.println(tagCount);
      AprilTagDatum tag;
      if(camera.readTag(tag)) // camera.readTag(tag))
      {
        // convert tag info into floats and scaled numbers
        data.id = tag.id;
        data.checksum = tag.checksum;
        data.w = ((float)tag.w) / 1000.0;
        data.h = ((float)tag.h) / 1000.0;
        data.rot = ((float)tag.rot) / 1000.0;
        data.x = ((float)tag.x) / 1000.0;
        data.y = ((float)tag.y) / 1000.0;
        data.z = ((float)tag.z) / 1000.0;
        data.rx = ((float)tag.x) / 1000.0;
        data.ry = ((float)tag.y) / 1000.0;
        data.rz = ((float)tag.z) / 1000.0;

        #ifdef TAG_DEBUG
          Serial.print(F("id: "));
          Serial.print(tag.id);
          Serial.print(F(" [w: "));
          Serial.print(tag.w);
          Serial.print(F(", h: "));
          Serial.print(tag.h);
          Serial.print(F(", rot: "));
          Serial.print(tag.rot);
          Serial.print(F(", x: "));
          Serial.print(tag.x);
          Serial.print(F(", y: "));
          Serial.print(tag.y);
          Serial.print(F(", z: "));
          Serial.print(tag.z);
          Serial.print(F(", rx: "));
          Serial.print(tag.rx);
          Serial.print(F(", ry: "));
          Serial.print(tag.ry);
          Serial.print(F(", rz: "));
          Serial.print(tag.rz);
          Serial.println(F("]"));
        #endif
      }
    }

    return tagCount;
}