
//The output for the serial monitor is PB0. Protocol is 1 start bit, 8 data bits, no parity, 1 stop bit.

void send_telemetry_data() {
  telemetry_loop_counter++; 


  //Send prøve-bytes
  
  if (telemetry_loop_counter == 1)telemetry_send_byte = 'F';                                //Send prøve info for å se om den mottar  
  if (telemetry_loop_counter == 2)telemetry_send_byte = 'C';


  //Send generell info
  
  if (telemetry_loop_counter == 3) {
    check_byte = 0;
    telemetry_send_byte = error;                                                            //Send feilen
  }
  if (telemetry_loop_counter == 4)telemetry_send_byte = flight_mode;                        //Send flight-mode
  if (telemetry_loop_counter == 5)telemetry_send_byte = battery_voltage * 10;               //Send spenningen
  if (telemetry_loop_counter == 6) {
    telemetry_buffer_byte = temperature;                                                    //Lagre temperaturen siden den kan endres
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send 1. byte
  }
  
  if (telemetry_loop_counter == 7)telemetry_send_byte = telemetry_buffer_byte >> 8;         //Send siste byte
  
  if (telemetry_loop_counter == 8)telemetry_send_byte = angle_roll + 100;                   //Send roll. Legg til 100 for å unngå negative tall
  if (telemetry_loop_counter == 9)telemetry_send_byte = angle_pitch + 100;                  //Send pitch. Legg til 100 for å unngå negative tall
  if (telemetry_loop_counter == 10)telemetry_send_byte = start;                             //Send start


  //Send høyden
  
  if (telemetry_loop_counter == 11) {
    if (start == 2) {                                                                       //Bare send når den flyr
      telemetry_buffer_byte = 1000 + ((ground_pressure - actual_pressure) * 0.0842);        //Send høyden hvis den flyr
    }
    else {
      telemetry_buffer_byte = 1000;                                                         //Send høyde 0 hvis den ikke flyr
    }
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send 1. variabel
  }
  if (telemetry_loop_counter == 12)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send siste byte


  //Send mengden thrust for å lette
  
  if (telemetry_loop_counter == 13) {
    telemetry_buffer_byte = 1500 + takeoff_throttle;                                        //Lagre take-off-throttle siden den kan endres før neste loop
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send 1. byte
  }
  if (telemetry_loop_counter == 14)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send siste byte
  


  //Send kompass info
  
  if (telemetry_loop_counter == 15) {
    telemetry_buffer_byte = angle_yaw;                                                      //Lagre vinkel siden den kan endres
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send 1. byte
  }
  if (telemetry_loop_counter == 16)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send siste byte

  if (telemetry_loop_counter == 17)telemetry_send_byte = heading_lock;                      //Send heading_lock



  //Send GPS data
  
  if (telemetry_loop_counter == 18)telemetry_send_byte = number_used_sats;                  //Antall satelitter brukt
  if (telemetry_loop_counter == 19)telemetry_send_byte = fix_type;                          //Hvordan type fix du har

  
  if (telemetry_loop_counter == 20) {
    telemetry_buffer_byte = l_lat_gps;                                                      //Lagre Latitude siden den kan endres før neste loop
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send 1. byte
  }
  if (telemetry_loop_counter == 21)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send 2. byte
  if (telemetry_loop_counter == 22)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Send 3. byte
  if (telemetry_loop_counter == 23)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Send siste byte
  
  
  if (telemetry_loop_counter == 24) {
    telemetry_buffer_byte = l_lon_gps;                                                      //Lagre Longitude siden den kan endres før neste loop
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send 1. byte
  }
  if (telemetry_loop_counter == 25)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send 2. byte
  if (telemetry_loop_counter == 26)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Send 3. byte
  if (telemetry_loop_counter == 27)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Send siste byte



  //endre til gimbal info

  if (telemetry_loop_counter == 28) {
    telemetry_buffer_byte = adjustable_setting_1 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 1.
  }
  if (telemetry_loop_counter == 29)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 1.
  if (telemetry_loop_counter == 30) {
    telemetry_buffer_byte = adjustable_setting_2 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 2.
  }
  if (telemetry_loop_counter == 31)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 2.
  if (telemetry_loop_counter == 32) {
    telemetry_buffer_byte = adjustable_setting_3 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 3.
  }
  if (telemetry_loop_counter == 33)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 3.




  if (telemetry_loop_counter == 34)telemetry_send_byte = check_byte;                        //Send the check-byte.



  //Etter 125 loops blir den reset slik at dronen sender info 4 ganger i sekundet

  if (telemetry_loop_counter == 125)telemetry_loop_counter = 0;                             //Reset variabelen slik at den begynner å telle på nytt igjen



  
  //Send the telemetry_send_byte via the serial protocol via ouput PB0.
  //Send a start bit first.
  
  if (telemetry_loop_counter <= 34) {
    check_byte ^= telemetry_send_byte;                                                        //Brukes for å sjekke om arduinon har mottatt alt den skal


    //Lag en start bit
    
    GPIOB_BASE->BSRR = 0b1 << 16;                                                             //PB0 = 0
    delayMicroseconds(104);                                                                   //104 us = 1/ 9600bps

    //Send byten
    
    for (telemetry_bit_counter = 0; telemetry_bit_counter < 8; telemetry_bit_counter ++) {    //Create a loop fore every bit in the
      if (telemetry_send_byte >> telemetry_bit_counter & 0b1){
        GPIOB_BASE->BSRR = 0b1 << 0;    //If the specific bit is set, set output PB0 to 1;
      }
      else {
        GPIOB_BASE->BSRR = 0b1 << 16;                                                      //If the specific bit is not set, reset output PB0 to 0;
      }
      delayMicroseconds(104);                                                                 //Delay 104us (1s/9600bps)
    }
    //Send a stop bit
    GPIOB_BASE->BSRR = 0b1 << 0;                                                              //PB0 = 1
  }
}
