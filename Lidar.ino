

/*
 * Brief : Calculate a Cyclic Redundancy Checks of 8 bits
 * Param1 : (*p) pointer to receive buffer
 * Param2 : (len) number of bytes returned by the TeraRanger
 * Return : (crc & 0xFF) checksum calculated locally
 */
uint8_t crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void setupLidar(){
  HWire.beginTransmission(0x31);
  HWire.endTransmission();
  delay(25);
}

void readLidar(){
  HWire.beginTransmission(0x31);  // Transmit to Evo Mini (THIS IS THE I2C BASE ADDRESS, CHANGE HERE IN CASE IT IS DIFFERENT)
  HWire.write(0x00);                     // Sends measure trigger byte
  HWire.endTransmission();               // Stop transmitting
  HWire.requestFrom(0x31, 3);     // Read back three bytes from Evo Mini (THIS IS THE I2C BASE ADDRESS, CHANGE HERE IN CASE IT IS DIFFERENT)

  buf[0] = HWire.read();                 // First byte of heightLidar
  buf[1] = HWire.read();                 // Second byte of heightLidar
  buf[2] = HWire.read();                 // Byte of checksum
  
  CRC = crc8(buf, 2);                   // Save the "return" checksum in variable "CRC" to compare with the one sent by the TeraRanger
    
  if (CRC == buf[2]) {                  // If the function crc8 return the same checksum than the TeraRanger, then:
    heightLidar = (buf[0]<<8) + buf[1];    // Calculate heightLidar in mm
  }
  else {
    Serial.println("CRC error!");
  }

  heightLidarTotal = heightLidarTotal * 0.8 + heightLidar * 0.2;
}


void pidLidar(){
  
  if (flight_mode >= 2/* && takeoff == 1*/) {
    if (desiredDistance == 0){
        desiredDistance = heightLidarTotal;                                                              //Sett setpunktet
      }
  
    heightError = desiredDistance - heightLidarTotal;
    pidLidarP = heightError * pidLidarGainP;
  
    pidLidarI += heightError * pidLidarGainI; 
    if(pidLidarI > 100){
      pidLidarI = 100;
    }
    else if(pidLidarI < -100){
      pidLidarI = -100;
    }
  
    pidLidarD = (heightError - heightErrorPrev) * pidLidarGainD;
  
  
    pidLidarTotal = pidLidarP + pidLidarI + pidLidarD;
    
    if(pidLidarTotal > 200){
      pidLidarTotal = 200;
    }
    else if(pidLidarTotal < -200){
      pidLidarTotal = -200;
    }
  
    heightErrorPrev = heightError;
  }
/*
  Serial.print("\n p gain");
  Serial.print(pidLidarP);
  Serial.print("\n i gain");
  Serial.print(pidLidarI);
  Serial.print("\n d gain");
  Serial.print(pidLidarD);
  Serial.print("\n pid distance gain : ");
  Serial.print(pidLidarTotal);
*/

  if (flight_mode < 2 && desiredDistance != 0){
    desiredDistance = 0;
    pidLidarI = 0;
    pidLidarTotal = 0;
  }
}
