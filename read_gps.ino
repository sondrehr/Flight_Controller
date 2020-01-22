
//Oppløsning:



void gps_setup(){

  Serial1.begin(9600);
  delay(250);


//setter variabler slik det er definert i databladet til gps-en
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial1.write(Disable_GPGSV, 11);                                                                           //Disable GPGSV meldinger som ikke er nødvendige
  delay(350); 

  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial1.write(Set_to_5Hz, 14);                                                                             //Set refresh rate til 5 Hz
  delay(350);   


  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1 };
                               
  Serial1.write(Set_to_57kbps, 28);                                                                         //Sett bRate til GPS-en til 57 Kbps
  delay(200);


  Serial1.begin(57600);                                                                                     //Sett baud-rate til 57 Kbps
  delay(200);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void read_gps() {

  //Lagrer den innkommende informasjonen i et array
  
  while (Serial1.available() && new_line_found == 0) {                                                   //Bli i loopen så lenge det er informasjon der
    char read_serial_byte = Serial1.read();                                                              //Les den neste variabelen
    if (read_serial_byte == '$') {                                                                       //Hvis det er en $ er det starten på en setning
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Sletter all den gamle informasjonen
        incomming_message[message_counter] = '-';                                                        
      }
      message_counter = 0;                                                                               //Vi vil begynne å skrive på begynnelsen av arrayet så dette er indeksen
    }
    else if (message_counter <= 99)message_counter ++;                                                   //Flytt den ett hakk bortover for hver char
    incomming_message[message_counter] = read_serial_byte;                                               //Skriv inn den nye bokstaven
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Hver linnje slutter med *
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //"aktiveres" når den mottar en ny linje
  
  if (new_line_found == 1) {                                                                            
    new_line_found = 0; 

                                                                                    
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //Hvis det ikke er noe GPS informasjon tilgjengelig
      digitalWrite(PC13, !digitalRead(PC13));                                                            //Endre LED for å vise signalstyrke
      
      l_lat_gps = 0;                                                                                     //Reset variabler hvis man mister dekning under ferdsel
      l_lon_gps = 0;                                                                                     //Brukes for å styre "GPS-lost" modus
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }

    

    //Sjekker om linja begynner med GA og at den har nok satelitter.
    //Legger så sammen alle tallene slik at vi får lengde- og breddekoordinatene
    //Trekker fra 48 siden det er ascii verdien til 0
    
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                   
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                    
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                     
      lat_gps_actual /= (long)6;                                                                         //minutter til grader deles det på 6
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                            
      lat_gps_actual /= 10;                                                                              //Del alt på 10

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                               
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                                
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                 
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                  
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                   
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                    
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                     
      lon_gps_actual /= (long)6;                                                                         
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                           
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                             
      lon_gps_actual /= 10;                                                                              




      //Sjekker hvilken "kvadrant" i verden dronen er i. Er viktig for D leddet i PD regulatoren
      
      if (incomming_message[28] == 'N'){
        latitude_north = 1;                                                                           
      }
      else {
        latitude_north = 0;                                                                         
      }

      if (incomming_message[42] == 'E'){
        longitude_east = 1;                                              
      }
      else {
        longitude_east = 0;                                                                     
      }

      

      //Antall satellitter
      
      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   
      number_used_sats += (int)incomming_message[47] - 48;                                               



      //Første gang GPS-en brukes

      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                             
        lat_gps_previous = lat_gps_actual;                                                              
        lon_gps_previous = lon_gps_actual;                                                               
      }



      //Regner ut det vi må legge til 9/10 looper for å øke frekvensen til en "simulert 50Hz"

      lat_gps_loop_add = (lat_gps_actual - lat_gps_previous) / 10.0;                              
      lon_gps_loop_add = (lon_gps_actual - lon_gps_previous) / 10.0;                             

      l_lat_gps = lat_gps_previous;                                                                     
      l_lon_gps = lon_gps_previous;


      lat_gps_previous = lat_gps_actual;                                                                 //Lagre til neste loop
      lon_gps_previous = lon_gps_actual;                                                                 //Lagre til neste loop


      gps_add_counter = 5;                                                                               //Tell ned 5 ganger. 5 * 4ms = 20 ms. 50Hz!
      new_gps_data_counter = 9;                                                                          //Antall "simulerte" målinger mellom hver måling
      lat_gps_add = 0;                                                                                   //Resetter variablene som skal legges til siden vi nå har gjort en ny måling
      lon_gps_add = 0;                                                                                 
      new_gps_data_available = 1;                                                                        //Det er ingen ny data tilgjengelig
    }

    

    //Hvis linja begynner med SA sjekker vi hvordan FIX vi har (ingenting, 2D, 3D)
    
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A'){
      fix_type = (int)incomming_message[9] - 48;
    }

  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //Etter 5 looper har det gått 20 ms
  
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                
    new_gps_data_available = 1;                                                                           //Hvis at der er data tilgjengelig
    new_gps_data_counter --;                                                                              //Teller ned så det kun er 9 iterasjoner
    gps_add_counter = 5;                                                                                  //Begynn telleren på nytt igjen

    lat_gps_add += lat_gps_loop_add;                                                                      //Legger til verdien som ble regnet ut over
    if (abs(lat_gps_add) >= 1) {                                                                          //Hvis verdien er større enn 1
      l_lat_gps += lat_gps_add;                                                                      //l_lat_gps kan ikke ha desimaler
      lat_gps_add -= lat_gps_add;                                                                    //Trekk fra slik at det kun er desimaltallet igjen
    }

    lon_gps_add += lon_gps_loop_add;                                                                   
    if (abs(lon_gps_add) >= 1) {                                                                         
      l_lon_gps += lon_gps_add;                                                                      
      lon_gps_add -= lon_gps_add;                                                                    
    }
  }



  //////////////////////////////////////////////////////////////////////////////////////////////////
  //Hvis det er data tilgjengelig
  //Regner ut PD loop osv...

  if (new_gps_data_available) {                                                                           
    if (number_used_sats < 8){
      digitalWrite(PC13, !digitalRead(PC13));                                                             //GPS signaler vises ved hjelp av PC13
    }
    else {
      digitalWrite(PC13, LOW);                                                                            
    }
    
    gps_watchdog_timer = millis();                                                                        //Reset timeren så passer på at GPS-en ikke mister signalet
    new_gps_data_available = 0;                                                                          



    //Den første gangen
    
    if (flight_mode >= 3 && waypoint_set == 0) {                                                          //flight mode 3 (GPS hold)
      waypoint_set = 1;                                                                                   
      l_lat_waypoint = l_lat_gps;                                                                         //Den nåværende latitude blir lagret
      l_lon_waypoint = l_lon_gps;                                                                         //Den nåværende longitude blir lagret
    }



    //Når et waypoint er satt

    if (flight_mode >= 3 && waypoint_set == 1) {


//Vi kan endre GPS-waypoint ved å bevege på pitch og yaw
////////////////////////////////////////////////////////////////////////////////////
    
      if (latitude_north) {
          l_lat_gps_float_adjust += 0.0013 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453))); 
        }
        else { 
          l_lat_gps_float_adjust -= 0.0013 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453))); 
        }

      if (longitude_east) {
          l_lon_gps_float_adjust += (0.0013 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); 
        }

        else {
          l_lon_gps_float_adjust -= (0.0013 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453);
        }
      

      if (l_lat_gps_float_adjust > 1) {                               //Gjør at dronen vil stoppe istedenfor å fortsette for alltid
        l_lat_waypoint ++;
        l_lat_gps_float_adjust --;
      }
      if (l_lat_gps_float_adjust < -1) {
        l_lat_waypoint --;
        l_lat_gps_float_adjust ++;
      }

      if (l_lon_gps_float_adjust > 1) {
        l_lon_waypoint ++;
        l_lon_gps_float_adjust --;
      }
      if (l_lon_gps_float_adjust < -1) {
        l_lon_waypoint --;
        l_lon_gps_float_adjust ++;
      }


                                                               
      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         
      gps_lat_error = l_lat_waypoint - l_lat_gps;
/*
      Serial.print(gps_lon_error);
      Serial.print("\t");
      Serial.print(gps_lat_error);
      Serial.print("\n");
*/


      //Bruker roterende array for å hindre støy i D leddet                                                         

      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                        
      gps_lat_rotating_mem [gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                      

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                      
      gps_lon_rotating_mem [gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         
      
      
      
      gps_rotating_mem_location++;                                                                        
      
      if ( gps_rotating_mem_location == 35){
        gps_rotating_mem_location = 0;                                                                    //Hold minnet innenfor gyldig område
      }


      gps_lat_error_previous = gps_lat_error;                                                             //Lagre feilen til neste loop
      gps_lon_error_previous = gps_lon_error;                                                             //Lagre feilen til neste loop



      //P = (float)gps_lat_error * gps_p_gain.
      //D = (float)gps_lat_total_avarage * gps_d_gain
      //Burde egt deles på 36, men kan bare gjøre gps_d_gain 36 ganger mindre
      
      gps_pitch_adjust_north = gps_lat_error * gps_p_gain + gps_lat_total_avarage * gps_d_gain;  
      gps_roll_adjust_north = gps_lon_error * gps_p_gain + gps_lon_total_avarage * gps_d_gain;


      
      //Må konvertere siden korreksjonen er kalkulert som hvis fronten var mot nord.


      gps_pitch_adjust = (gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + (gps_roll_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_roll_adjust = (gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + (gps_pitch_adjust_north * cos((angle_yaw + 90) * 0.017453));



      //Begrens pådraget

      if (gps_roll_adjust > 250) gps_roll_adjust = 250;
      if (gps_roll_adjust < -250) gps_roll_adjust = -250;
      if (gps_pitch_adjust > 250) gps_pitch_adjust = 250;
      if (gps_pitch_adjust < -250) gps_pitch_adjust = -250;
    }
  }



  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //Sjekker om det tar for lang tid mellom hver gang GPS-en mottar signal
  //Hvis det tar for lang tid: slå av GPS hold

  if (gps_watchdog_timer + 1000 < millis()) {                                                             //Hvis det ikke får signal innen en viss tid sett feil lik 4
    if (flight_mode >= 3 && start > 0) {                                                                  
      flight_mode = 2;                                                                                   
      error = 4;                                                                                          
    }
  }



  /////////////////////////////////////////////////////////////////////////////////////////////////// 
  //Reseter alle verdier slik at det er klart til neste aktivering
  
  if (flight_mode < 3 && waypoint_set > 0) {                                                              
    gps_roll_adjust = 0;                                                                                
    gps_pitch_adjust = 0;                                                                                
    if (waypoint_set == 1) {                                                                              
      gps_rotating_mem_location = 0;                                                                      
      waypoint_set = 2;                                                                                   //Det roterende minnet er ikke tømt
    }
    gps_lon_rotating_mem [gps_rotating_mem_location] = 0;                                                 
    gps_lat_rotating_mem [gps_rotating_mem_location] = 0;                                                 
    gps_rotating_mem_location++;                                                                          
    if (gps_rotating_mem_location == 36) {                                                                //Når arrayet er tømt, sett waypoint lik 0
      waypoint_set = 0;                                                                              
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;                                                                          ////Reset variablene brukt i PD regulatoren
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
      l_lat_waypoint = 0;
      l_lon_waypoint = 0;
    }
  }
}
