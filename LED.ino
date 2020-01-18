
void red_led(int8_t level) {
  digitalWrite(PA12, level);
}
void green_led(int8_t level) {
  digitalWrite(PA11, level);
}

//Rød LED
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void error_signal() {
  if (error >= 100) {
    red_led(HIGH);                                                                         //feil > 100, rødt lys alltid på
  }
  else if (error_timer < millis()) {                                                       
    error_timer = millis() + 250;                                                          //Må vente 250 milli-sekunder før neste feil-blink
    if ( error_counter > error + 3) {                                          //Vente 750 millisekunder før neste feilmelding
      error_counter = 0;                                                                 
    }
    if (error_counter < error && error_led == 0) {                        //Hvis feilmeldingen ikke er ferdig og LED er av
      red_led(HIGH);                                                                   
      error_led = 1;                                                                     
    }
    else {                                                                                 //Slå av lyset og tell en høyere
      red_led(LOW);                                                                       
      error_counter++;                                                                 
      error_led = 0;                                                                     
    }
  }
}

//Grønn LED
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void flight_mode_signal() {
  if (flight_mode_timer < millis()) {                                                     
    flight_mode_timer = millis() + 250;                                                    //Må vente 250 milli-sekunder før neste blink
    if (flight_mode_counter > flight_mode + 3) {
      flight_mode_counter = 0;                                                             //If there is an error to report and the error_counter > error +3 reset the error.
    }
    if (flight_mode_counter < flight_mode && flight_mode_led == 0){     //Hvis feilmeldingen ikke er ferdig og LED-en er av
      green_led(HIGH);                                                                   
      flight_mode_led = 1;                                                               
    }
    else {                                                                                 //Slå av lyset og tell en opp
      green_led(LOW);                                                                    
      flight_mode_counter++;                                                           
      flight_mode_led = 0;                                                             
    }
  }
}
