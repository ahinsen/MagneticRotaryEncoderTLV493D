#Basic API for using the TLV493D-A1B6 as a rotary encoder
The header file contains:
 - A data structure for sensor settings, status and measured values
 - Functions for initializing the device, reading measurements and convert them to angle values

#Environment I use:
 - Teensy3.2 
 - TLV493D-A1B6 connected to standard i2c pins (18,19) and power connected to pin 21
 - Arduino 1.8.3 extended with teensyduino
 
#API functions:
Each function manipulates a data structure declared as a common variable of type TLV493D_t  
 - initSensor(TLV493D_t *x). Before calling this function, the device settings should be set in x
 - softReset(TLV493D_t *x).  To be used in case the measured values are invalid
 - readMeasurement(TLV493D_t *x). Delivers measured values in the "measured values" and "device status" fields of x
 - read10Measurements(TLV493D_t *x). Cumulates the values of 10 consecutive measurements (assuming mode 3:masterControlled)
                                     I found cumulation (averaging) necessary to raise accuracy.
 - angle(int16_t x,int16_t y, TLV493D_t *s). Calculates the angle of the magnetic field
 - measurements2str(TLV493D_t *x). String representation of the measured values (and status values if not OK)

 #Example
This code displays min/max magnetic values in each direction while a magnet is rotated a full circle near the device.
This is useful for collecting calibration values for the angle() function in TLV493D-A1B6.h
Note: This example uses the cumulative read functionality, so the displayed values will be in the range of -20470 to 20480 in stead of -2047 to 2048 values provided by the device.
 