#include "A1332.h"
#include "Arduino.h"
#include "Wire.h"

float A1332_GetAngle( void ) {
    int msb, lsb, raw;

    Wire.beginTransmission(A1332_ADDR);
    if (Wire.endTransmission() != 0)
        return A1332_ANGVAL_ERR;
    
    if (Wire.requestFrom(A1332_ADDR, 2) != 2)
        return A1332_ANGVAL_ERR;

    msb = Wire.read();
    lsb = Wire.read();

    // check
    if(msb == -1 || lsb == -1)
        return A1332_ANGVAL_ERR;

    // decode (12-bit resolution, raw value ranges from 0 to 4095)
    raw = ((msb & 0x0F) << 8) + (lsb & 0xFF);

    // convert to angle (0 to 360 degrees)
    return(180 - (float)( ((double)raw * (double)360.0) / (double)4095 ) );
}
