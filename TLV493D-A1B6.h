/* TLV493D-A1B6 3D Magnetic sensor over i2c interface
 * This file contains:
 * - A data structure for sensor settings status and measured values
 * - Functions for initializing the device, reading measurements and convert them to angle values
*/
#ifndef _TLV493D_A1B6_
#define _TLV493D_A1B6_
#include <i2c_t3.h>
#define DEBUG 1
// device status and settings
typedef struct{
	char name[10];              // Display name of the sensor (used by measurement2str)
	//Device settings (see TLV493D-A1B6 datasheet)
	uint8_t w[4]={0,0,0,0};     // write-registers: values to be written to the device
	uint8_t r[10]={0,0,0,0,0,0,0,0,0,0};// read registers: values read from the device
	uint8_t	p=0;				// Parity bit: 0 or 1 
	uint8_t iicaddr=0; 		    // I2C Address bits: 0,1,2, or 3 (see i2cAddr() )
	uint8_t	interrupt=1;        // Interrupt output enabled: 1 disabled 0
	uint8_t mode=0;             // 0:powerDown, 1:lowPower, 2:Fast, 3:masterControlled
	uint8_t	t=0;                // Temperature measurement enabled:0 Disabled:1
	uint8_t	lp=0;               // Low Power Period: 0:100ms (ultra low pwr) 1:12ms
	uint8_t	pt=1;               // Parity test enabled: 0:no 1:yes
	uint8_t powerOnSDA=1;       // SDA level during power on (0:low 1: high)(see i2cAddr() )
	uint8_t powerPin=20;        // the pin that serves as i2c power
	//Measured values
	signed int bx,by,bz;        // Magnetic values NOT CONVERTED TO mT -2048 to 2047
	signed int temp;            // Temperature converted to 0.1C (integer value is 10 times the C value)
    //Status info from the device
	uint8_t frm=100;            // frame counter (2bit). incremented after each conversion
	uint8_t lastFrm=100;        // previous value of frame counter (to check delta)
	uint8_t ch=1;               // channel. 0 means reading values are meaningful
	uint8_t pd=1;               // power down. 1:conversion is in progress, 0: data ready
    //communication and general status
	uint8_t i2cStat;            // result of the last i2c operation
	uint8_t valid=0;            // validity of the last reading (based on frm, lastFrm, ch, pd and i2cStat)
	//calibration values for angle calculation (see angle())
	int16_t xmin,xmax,ymin,ymax,zmin,zmax; //range of measured values over a full 360 degree rotation
	uint16_t CPR;               // Code per rotation: angle() will return values between 0 and(CPR-1) 
} TLV493D_t;
// --------------------------------"Private" functions ------------------------------------------------------

/* Device address determination
 * Returns the 7 bit device address without the R/W bit, based on the device settings
 */
uint8_t i2cAddr(TLV493D_t *x){ 
	const uint8_t  addresses[8]={0x1F,0x1B,0x0F,0x0B,0x5E,0x5A,0x4E,0x4A};
	return addresses[(x->powerOnSDA<<2)|(x->iicaddr)];
}

/* CONVERSIONS of measured values 
 * Calculates Bx,By,Bz values as integer values between -2047 and 2048
 * Calculates temperature value in 0.1C (ten times the real celsius value)
 * Updates device status values and the general validity flag
 * Two versions: simple and cumulative reading
 */
void addReading2measurement(TLV493D_t *x){// update measurement values and settings based on read register values
	x->bx+=	(x->r[0]>>7)*(-2048)+ (x->r[0]&0x7F)*16 + (x->r[4]>>4);
	x->by+=	(x->r[1]>>7)*(-2048)+ (x->r[1]&0x7F)*16 + (x->r[4]&0x0F);
	x->bz+=	(x->r[2]>>7)*(-2048)+ (x->r[2]&0x7F)*16 + (x->r[5]&0x0F);
	x->temp+=(((x->r[3]>>7)*(-2048)+((x->r[3]&0x70)>>4)*256+(x->r[6]))-340)*11;// convert to C value
    x->lastFrm=x->frm;
	x->frm=	(x->r[3]&0x0C)>>2;
	x->ch=	(x->r[3]&0x03);
	x->pd=	(x->r[5]&0x10)>>4;
	x->valid=(x->ch==0 && x->pd==0 && x->i2cStat==0 && x->frm!=x->lastFrm);
}
void reading2measurement(TLV493D_t *x){
	x->bx=0;
	x->by=0;
	x->bz=0;
	x->temp=0;
	addReading2measurement(x);
}
/* Prepare write register register values based on settings and read reg values
 * before writing them to the device
 */
void prepareWriting (TLV493D_t *x){
	x->w[1]=(x->p)<<7 | (x->iicaddr)<<5 | ((x->r[7])&0x18) | (x->interrupt)<<2 | x->mode;
	x->w[2]=x->r[8];
	x->w[3]=(x->t)<<7 | (x->lp)<<6 | (x->pt)<<5 | ((x->r[9])&0x1F);
	// check-set parity
	uint8_t p= x->w[0] ^ x->w[1] ^ x->w[2] ^ x->w[3];
	p = (p&0x0F) ^ (p>>4);
	p = (p&0x03) ^ (p>>2);
	p = (p&0x01) ^ (p>>1);
	if (!p) x->w[1]^=0x80;
}
/* Functions for i2c communication
 */
const char *i2cStatStr[12]={ 
  "I2C_OK",      "I2C_SENDING", "I2C_SEND_ADDR","I2C_RECEIVING","I2C_TIMEOUT", "I2C_ADDR_NAK",
  "I2C_DATA_NAK","I2C_ARB_LOST","I2C_BUF_OVF",  "I2C_SLAVE_TX", "I2C_SLAVE_RX","I2C_SW_ERROR"
};

void i2cRead(uint8_t address, uint8_t length,TLV493D_t *x){// --------- I2C READ --------------
  if (length<1 || length>10) {
	  x->i2cStat=100;
	  return;
  }
  Wire.requestFrom(address,(size_t)length,I2C_STOP); 
  for (uint8_t i=0;i<length;i++){
    x->r[i]=Wire.readByte();
    x->i2cStat=Wire.status();
    if (x->i2cStat!=I2C_WAITING) {
      Wire.resetBus();
      return; 
    }
  }
}
char * settings2str(TLV493D_t *x);
void i2cWrite(uint8_t address,TLV493D_t *x){              // --------- I2C WRITE --------------
	prepareWriting(x);
	Wire.beginTransmission(address); 
	for (uint8_t i=0;i<4;i++) Wire.write(x->w[i]); 
	Wire.endTransmission();
	x->i2cStat=Wire.status();
	if (DEBUG){
		char buf[100];
		if (x->i2cStat){
			sprintf(buf,"%s:TLV493D writing settings to %X:%s\n",x->name,address,i2cStatStr[x->i2cStat]);
			Serial.print(buf);
		}
		else Serial.print(settings2str(x));
	}
	delay(1);
}
/* --------------------------------"Public" functions ------------------------------------------------------
 * The following functions are to be used by the application:
 * - initSensor(TLV493D_t *x). Before calling this function, the device settings should be set in x
 * - softReset(TLV493D_t *x).  To be used in case the measured values are invalid
 * - readMeasurement(TLV493D_t *x). Delivers measured values in the "measured values" and "device status" fields of x
 * - read10Measurements(TLV493D_t *x). Cumulates the values of 10 consecutive measurements (assuming mode 3:masterControlled)
 *                                     for averaging purposes.
 * - angle(int16_t x,int16_t y, TLV493D_t *s). Calculates the angle of the magnetic field
 * - measurements2str(TLV493D_t *x). String representation of the measured values (and status values if not OK)
 */
void initSensor(TLV493D_t *x){            // ----------------------------- INIT SENSOR ---------------------
  digitalWrite(x->powerPin, LOW);		  //Power down
  pinMode(x->powerPin, OUTPUT);
  delay(1);
  pinMode(18, OUTPUT); 					  //set SDA
  digitalWrite(18,x->powerOnSDA);
  digitalWrite(x->powerPin,HIGH);		  //Power up
  delay(1);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);// Init i2c functionality
  Wire.setDefaultTimeout(1000);
  uint8_t tmpAddr=(x->powerOnSDA ? 0x5E : 0x1F);//(iicaddr is 00 at this time by default)
  i2cRead(tmpAddr,10,x);				  //read all registers 
  if (DEBUG){
	char buf[100];
    sprintf(buf,"%s:TLV493D Init reading 10 bytes from %X:%s\n",x->name,tmpAddr,i2cStatStr[x->i2cStat]);
    Serial.print(buf);
  }
  x->frm=100;
  i2cWrite(tmpAddr,x);
}

void softReset(TLV493D_t *x){               //-------------------------------- SOFT RESET------------------
  i2cRead(0,1,x);
  delay(1);
  x->frm=100;
  i2cWrite(i2cAddr(x),x);
}

void readMeasurements(TLV493D_t *x){        //-------------------------------- READ MEASUREMENTS ---------
  i2cRead(i2cAddr(x),7,x);
  reading2measurement(x);
}

void read10Measurements(TLV493D_t *x){
  uint16_t delay=x->t?150:200; //delay between measurements depends if temperature is enabled or not
  elapsedMicros t;
  readMeasurements(x);
  for (uint8_t i=0; i<9; i++){
    i2cRead(i2cAddr(x),7,x);
	t=0;
    addReading2measurement(x);
    while (t<delay);
  }
}
/* Convert measured values to an angle in the pane defined by any two of the three axes
 * Parameters: the measured values of the two selected axes, and a reference to a  TLV493D_t variable (for the calibration values)
 * Returns: An integer value between 0 and CPR-1 corresponding to the measured angle between 0 and 360 degrees (or 0 - 2PI radian)
 * The angle is measured from the axis of the first parameter incrementing towards the direction of the 2nd parameter's axis
 * Example: TLV493D_t x; angle(x.bz, x.by, &x); will return 0 for a Z direction magnetic field and CPR/4 for Y direction magnetic field 
*/
uint16_t angle(int16_t x,int16_t y, TLV493D_t *s){
  int xHalfRange=(s->xmax-s->xmin)/2;
  int yHalfRange=(s->ymax-s->ymin)/2;
  int xMid=s->xmin+xHalfRange;
  int yMid=s->ymin+yHalfRange;
  float xNorm=((float)(x-xMid))/((float)xHalfRange);
  float yNorm=((float)(y-yMid))/((float)yHalfRange);
  float a=atan(yNorm/xNorm)*(s->CPR)/6.283;
  if (xNorm<0)a+=(s->CPR)/2;
  if (a<0) a+=(s->CPR);
  return (uint16_t)a;
}

char * measurements2str(TLV493D_t *x){
	static char buf[100];
	if(x->valid)sprintf(buf,"%s:Bx:%5d By:%5d Bz:%5d Temp:%5d (valid)\n",x->name,x->bx,x->by,x->bz,x->temp);
	else sprintf(buf,"%s:Bx:%5d By:%5d Bz:%5d Temp:%5d Frame/last:%d/%d Ch:%d PD:%d i2cAddr/Stat:%X/%s\n",x->name,x->bx,x->by,x->bz,x->temp,x->frm,x->lastFrm,x->ch,x->pd,i2cAddr(x),i2cStatStr[x->i2cStat]);
	return &buf[0];
}
char * settings2str(TLV493D_t *x){
	static char buf[100];
	sprintf(buf,"%s settings:Parity:%d Addr:%X Int:%d Mode:%d Temp:%d LP:%d PT:%d\n",x->name,x->p,i2cAddr(x),x->interrupt,x->mode,x->t,x->lp,x->pt);
	return &buf[0];
}

#endif