
void calibrate_compass() {
  
  compass_calibration_on = 1;                                                
  
  red_led(HIGH);                                                             
  green_led(LOW);                                                           
 /*
  while (channel_2 > 1100) {                                                
    
    //send_telemetry_data();                                                 
    
    delayMicroseconds(3700);                                                 
    read_compass();                                                          

    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;                            //Les maks og min på hver akse for å skalere alle riktig
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
    

    for (int i = 0; i < 6; i++){
      Serial.print(compass_cal_values[i]);
      Serial.print("   ");
    }
    Serial.print("\n");
  }
  */

  //målte verdier så jeg slipper å kalibrere kompasset hver gang det starter.

  compass_cal_values[0] = -724;
  compass_cal_values[1] = 463;
  compass_cal_values[2] = -682;
  compass_cal_values[3] = 503;
  compass_cal_values[4] = -546;
  compass_cal_values[5] = 625;
  
  compass_calibration_on = 0;                                              

  for (j = 0; j < 6; j ++){
    EEPROM.write(0x10 + j, compass_cal_values[j]);
  }

  setup_compass();                                                           //Initialiser kompasset og set verdier
  read_compass();                                                            //Les data fra kompasset
  
  angle_yaw = actual_compass_heading;                                        //Sett den initielle retningen

  red_led(LOW);
  
  for (j = 0; j < 15; j ++) {
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }

  loop_timer = micros();                                                     //Sett timeren
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calibrate_level() {
  
  level_calibration_on = 1;
/*
  while (channel_2 < 1100) {
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    delay(10);
  }
*/  
  red_led(HIGH);
  green_led(LOW);

  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  j = 0;
  
  for (i = 0; i < 64; i ++) {
    send_telemetry_data();                                                  //Send data til Tx
    gyro_signal();
    acc_pitch_cal_value += acc_x;
    acc_roll_cal_value += acc_y;
    
    if (abs(acc_x*9.81/4096) > 2){                                          //Hindrer at dronen står skjevt under kalibreringen
      j = 1;
    }
    if (abs(acc_y*9.81/4096) > 2){
      j = 1;
    }
    delayMicroseconds(3700);
  }

  acc_pitch_cal_value /= 64;                                                        //Finner gjennomsnittet
  acc_roll_cal_value /= 64;
  
  red_led(LOW);
  
  if (j < 1) {
    EEPROM.write(0x16, acc_pitch_cal_value);
    EEPROM.write(0x17, acc_roll_cal_value);
    for (i = 0; i < 15; i ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    error = 0;
  }
  
  else {
    error = 3;
  }

  level_calibration_on = 0;                                                        //signaliserer at kalibreringen har stoppet
  gyro_signal();                                                                   //Tar en siste lesning 

  loop_timer = micros();                                                           //Set the timer for the next loop.
}
