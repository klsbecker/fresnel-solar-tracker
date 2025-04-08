#ifndef DEVICE_A1332_H_
#define DEVICE_A1332_H_

/*
  "The default slave address for the A1332 is 00011xx, where
  the two LSB bits are set by the package pins (SA1 and SA0)
  being tied high or low.
  ---------------------------
    SA1   SA0    Addr. Value

    BYP   BYP     00011 11
    BYP   GND     00011 10
    GND   BYP     00011 01
    GND   GND     00011 00
  ---------------------------
  Refer to the INTF field in the EEPROM Description and Programming
  section for alternative, programmatic settings."
*/
#define A1332_ADDR 0x0C
#define A1332_ANGVAL_ERR (float)-1.0

float A1332_GetAngle(void);

#endif