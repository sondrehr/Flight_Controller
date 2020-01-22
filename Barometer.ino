


void read_barometer() {
  barometer_counter ++;

  //det er 3 looper her hvor en total loop tar 12 ms
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

    barometer_counter = 0;                                                                                    //Reset telleren slik at den begynner på nytt
    
    //Her er et roterende array brukt som et slags numerisk lavpassfilter
    
    if (manual_altitude_change == 1){
      pressure_parachute_previous = actual_pressure * 10;                                                     //Når vi endrer høyden manuelt er "parachute mode" slått av
    }
    
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  
    
    pressure_parachute_previous = actual_pressure * 10;                                                       
    
    parachute_rotating_mem_location++;                                                                        
   
    if (parachute_rotating_mem_location == 30){
      parachute_rotating_mem_location = 0;                                                                    //Hold verdien innenfor et gyldig område
    }




    if (flight_mode >= 2 && takeoff == 1) {                                                                   //Hvis dronen flyr og er i altitude hold
      if (pid_altitude_setpoint == 0){
        pid_altitude_setpoint = actual_pressure;                                                              //Sett setpunktet
      }
      
      //Hvis vi vil øke/senke høyden kan vi ta throttle høyere enn verdiene under. Da vil altitude hold settes på pause slik at dronen kan stige/synke. 
      //Når dronen er ved ønsket høyde tar vi throttle innenfor rangen under og da aktiveres altitude hold igjen
      
      manual_altitude_change = 0;                                                    
      manual_throttle = 0;                                                           
      
      if (channel_3 > 1700) {                                                        
        manual_altitude_change = 1;                                                  //Sett lik 1 for å vise at høyden endres
        pid_altitude_setpoint = actual_pressure;                                     //Sett settpunktet lik høyden vi er på så output av PID er lik 0
        manual_throttle = (channel_3 - 1750) / 3;                                    //Begrens throttle så den ikke kan stige eller synke for fort
      }
      
      if (channel_3 < 1550) {                                                        
        manual_altitude_change = 1;                                                  //Sett lik 1 for å vise at høyden endres
        pid_altitude_setpoint = actual_pressure;                                     //Sett settpunktet lik høyden vi er på så output av PID er lik 0
        manual_throttle = (channel_3 - 1600) / 5;                                    //Begrens throttle så den ikke kan stige eller synke for fort
      }


//PID
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      pid_altitude_input = actual_pressure;                                          
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //regn ut feilen mellom den fsktiske høyden og ønsket høyde
      
      
      pid_error_gain_altitude = 0;                                                   
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //Hvis absoluttverdien av feilen er større enn 10
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 
        if (pid_error_gain_altitude > 3){
          pid_error_gain_altitude = 3;                                               //P-gains er begrenset til 3 for å hindre for stort pådrag
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

    //Variabler må resetes hvis vi går ut av altitude hold slik at det ikke blir rykkete hvis den aktiveres igjen
    
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        
      pid_altitude_setpoint = 0;                                                     //Reset PID
      pid_output_altitude = 0;                                                       //Reset PID
      pid_i_mem_altitude = 0;                                                        //Reset PID
      manual_throttle = 0;                                                           
      manual_altitude_change = 1;                                                    
    }
  }
}
