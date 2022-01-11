

//initialiserer registrene
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setupIMU(int16_t address) {
  HWire.beginTransmission(address);                        
  HWire.write(0x6B);                                            //PWR_MGMT_1 for å nullstille gyro
  HWire.write(0x00);                                            //00000000
  HWire.endTransmission();                                

  HWire.beginTransmission(address);                     
  HWire.write(0x1B);                                            //GYRO_CONFIG for å sette måleområdet
  HWire.write(0x08);                                            //00001000 (500 grader/s).
  HWire.endTransmission();                                  

  HWire.beginTransmission(address);                       
  HWire.write(0x1C);                                            //ACCEL_CONFIG for å sette måleområdet.
  HWire.write(0x16);                                            //00010000 (+/- 8g)
  HWire.endTransmission();                                  

  HWire.beginTransmission(address);                       
  HWire.write(0x1A);                                            //CONFIG for å bruke et digitalt lavpassfilter
  HWire.write(0x03);                                            //00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();                                  

  acc_pitch_cal_value  = EEPROM.read(0x16);
  acc_roll_cal_value  = EEPROM.read(0x17);
}

//Kalibreringen av gyroen
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calibrate_gyro() {
  
  gyro_roll_cal = 0;
  gyro_pitch_cal = 0;
  gyro_yaw_cal = 0;  

  for (cal_int = 0; cal_int < 2000; cal_int++) {                                  //tar 2000 lesninger av signalet
    if (cal_int % 50 == 0){
      digitalWrite(PA12, !digitalRead(PA12));                                       //slår LED av og på under calibreringen
    }
    readIMU();                                                          
    gyro_roll_cal += gyro_roll;                                                  
    gyro_pitch_cal += gyro_pitch;                                                  
    gyro_yaw_cal += gyro_yaw;                                                     
    delay(4);                                                                       //simuler 250 Hz frekvens
  }

  gyro_roll_cal /= 2000;                                                            //finn gjennomsnittet
  gyro_pitch_cal /= 2000;                                                           
  gyro_yaw_cal /= 2000;                                                           

  red_led(LOW);                                                                     //Sett rød LED lav
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readIMU() {
  //HWire.beginTransmission(0x69);
  HWire.beginTransmission(0x69);                      
  HWire.write(0x3B);                                           
  HWire.endTransmission();                                    
  HWire.requestFrom(0x69, 14);                                 //Be om 14 bytes
  acc_x = HWire.read() << 8 | HWire.read();               
  acc_y = HWire.read() << 8 | HWire.read();                 
  acc_z = HWire.read() << 8 | HWire.read();                 
  temperature = HWire.read() << 8 | HWire.read();        
  gyro_roll = HWire.read() << 8 | HWire.read();             
  gyro_pitch = HWire.read() << 8 | HWire.read();           
  gyro_yaw = HWire.read() << 8 | HWire.read();
  /*    
  acc_x *= -1;  
  gyro_yaw *= -1;                                              //Snu om på yaw aksen
  */
  acc_y *= -1;
  gyro_yaw *= -1;
  gyro_pitch *= -1;
  gyro_roll *= -1;
  

  if (level_calibration_on == 0) {                             //Trekk fra den feilen kalkulert i kalibreringen av gyroen
    acc_x -= (acc_pitch_cal_value);                              
    acc_y -= (acc_roll_cal_value);                              
  }


  if (cal_int == 2000) {
    gyro_roll -= gyro_roll_cal;                                //trekker fra drift i gyroen
    gyro_pitch -= gyro_pitch_cal;                         
    gyro_yaw -= gyro_yaw_cal;
  }
}
