/* Example code for TLV493D-A1B6.h
 * This code displays min/max magnetic values in each direction 
 * while a magnet is rotated (manually) a full circle near the device 
 * To be used for collecting calibration values for the angle() function in TLV493D-A1B6.h
 */

#include "TLV493D-A1B6.h"

TLV493D_t s1; 

void setup() {
  s1.powerPin=21;// set your pin connected to device power
  s1.mode=3;     // Master controlled
  s1.t=1;        // disable temperature measurement 
  s1.interrupt=0;// no interrupt
  strcpy(s1.name,"S1");
  Serial.begin(9600);
  while (!Serial);
  initSensor(&s1);
}

int16_t xmin=20480,ymin=20480,zmin=20480;
int16_t xmax=-20480,ymax=-20480,zmax=-20480;
uint16_t i=0;
uint8_t chgd=0;
char buf[100];
void loop() {
    read10Measurements(&s1);
    if (!s1.valid) {
      Serial.println("Reading error");
      delay(1000);
    } else {
      if (s1.bx<xmin) {xmin=s1.bx;chgd=1;};
      if (s1.bx>xmax) {xmax=s1.bx;chgd=1;};
      if (s1.by<ymin) {ymin=s1.by;chgd=1;};
      if (s1.by>ymax) {ymax=s1.by;chgd=1;};
      if (s1.bz<zmin) {zmin=s1.bz;chgd=1;};
      if (s1.bz>zmax) {zmax=s1.bz;chgd=1;};
    }
    if (chgd) {
      sprintf(buf,"%5d X[min max]:[%6d %6d] Y[min max]:[%6d %6d] Z[min max]:[%6d %6d]\r\n",i++,xmin,xmax,ymin,ymax,zmin,zmax);
      Serial.print(buf);
    }
    delay(100);
    chgd=0;
}
