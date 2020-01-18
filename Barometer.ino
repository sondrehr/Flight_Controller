
//Oppløsning:




void read_barometer() {
  barometer_counter ++;

  //det er 3 looper her hvor en loop går hvert 12 ms
  //Henting av data tar 9 ms fra MS5611 så derfor passer det å hente dataen hver 3. loop

  if (barometer_counter == 1) {                                         
    if (temperature_counter == 0) {                                             
      
      HWire.beginTransmission(0x77);                                  
      HWire.write(0x00);                                                        //Send en 0 for å indikere at vi ønsker dataen
      HWire.endTransmission();                                             
      HWire.requestFrom(0x77, 3);                                               //Hent temperaturen fra MS5611        
                                  
     
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];       //Lagre temperaturen i en roterende array for å forhindre temperaturendringer
      raw_temperature_rotating_memory[average_temperature_mem_location] = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      
      if (average_temperature_mem_location == 5){                               //Det er 5 målinger. Holder den innenfor gyldig området
        average_temperature_mem_location = 0;
      }
      raw_temperature = raw_average_temperature_total / 5;                      //Det er 5 målinger. Finner gjennomsnittet
    }
    
    else {
      
      //Hent trykk-data fra MS5611
      HWire.beginTransmission(0x77);                                            
      HWire.write(0x00);                                                        //Send 0 for å indikere at vi ønsker dataen som er queuet
      HWire.endTransmission();                                                  
      HWire.requestFrom(0x77, 3);                                               //hent 3 bytes  
      
      raw_pressure = HWire.read() << 16 | HWire.read() << 8 | HWire.read();     
    }

    //////////////////////////

    temperature_counter ++;                                                    
     
    if (temperature_counter == 20) {                                            //1 av 20 ganger henter den temperatur-data
      temperature_counter = 0;                                                  //Reset variabelen slik at koden over henter ut riktig data
     
      HWire.beginTransmission(0x77);                                
      HWire.write(0x58);                                                        //0x58 = temperatur-data
      HWire.endTransmission();                                                  
    }
    
    else {                                                                      //19 av 20 ganger requester den trykk-data#
      
      HWire.beginTransmission(0x77);                                
      HWire.write(0x48);                                                        //0x48 = trykk-data
      HWire.endTransmission();                                               
    }
  }

/////////////////////////////////////////////////////////////

  if (barometer_counter == 2) {                                                 
    
    dT = raw_temperature - (C[5] * pow(2, 8));                                          
    
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);                                              //Regn ut trykket i millibar slik det er forklart i datasheet-et til MS5611

    
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //bruker et roterende array for å få gjevne målinger
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                               
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          
    pressure_rotating_mem_location++;                                                                         
    
    if (pressure_rotating_mem_location == 20){
      pressure_rotating_mem_location = 0;                                                                     //Det er 20 målinger
    }
    
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //FInn gjennomsnittet av de 20 målingene

    
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;         //Bruker et komplementær filter
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Finner forskjellen
    
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //Den "sakte" skal følge etter den kjappe.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //Dette hindrer at den følger den for nøyaktig og dermed blir ustabil


    if (actual_pressure_diff > 1 || actual_pressure_diff < -1){
      actual_pressure_slow -= actual_pressure_diff / 6.0;
    }
    actual_pressure = actual_pressure_slow;                                                                   //Trykket som blir brukt i loopen

/*
    Serial.print("\n");
    Serial.print(actual_pressure);
    Serial.print("\t");
    Serial.print(actual_pressure/100);
  */
  }

/////////////////////////////////////////////////////////////

  if (barometer_counter == 3) {                                                                               

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    
    if (manual_altitude_change == 1){
      pressure_parachute_previous = actual_pressure * 10;                                                     //During manual altitude change the up/down detection is disabled.
    }
    
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30){
      parachute_rotating_mem_location = 0;                                                                    //Hold verdien innenfor et gyldig område
    }




    if (flight_mode >= 2 && takeoff == 1) {                                                                   //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0){
        pid_altitude_setpoint = actual_pressure;                                                              //If not yet set, set the PID altitude setpoint.
      }
      
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      
      if (channel_3 > 1700) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1750) / 3;                                    //To prevent very fast changes in hight, limit the function of the throttle.
      }
      
      if (channel_3 < 1550) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1600) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.



      
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3){
          pid_error_gain_altitude = 3;                                               //To prevent extreme P-gains it must be limited to 3.
        }
      }

      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      
      if (pid_i_mem_altitude > pid_max_altitude){
        pid_i_mem_altitude = pid_max_altitude;
      }
      else if (pid_i_mem_altitude < pid_max_altitude * -1){                          //Anti wind-up
        pid_i_mem_altitude = pid_max_altitude * -1;
      }
      
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;

      
      if (pid_output_altitude > pid_max_altitude){
        pid_output_altitude = pid_max_altitude;
      }
      else if (pid_output_altitude < pid_max_altitude * -1){                        //Begrensning i pådrag
        pid_output_altitude = pid_max_altitude * -1;
      }
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
  }
}
