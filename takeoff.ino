
void start_stop_takeoff() {
  
  if (channel_3 < 1050 && channel_4 < 1050 && channel_2 > 1050 && start != 3) {      //starte modus 2
    start = 1;                                                                       //Starte motoren: nederst til venstre med venstre stikke
  }

 /*                                                                                 
  if (channel_3 < 1050 && channel_4 < 1050 && channel_2 < 1050 && channel_1 < 1050){ //starte modus 3
    start = 3;                                                                       //For å ta auto-takeoff. Begge stikkene nederst til venstre
  }
*/
  
  if (start == 1 && channel_4 > 1350) {                                            //Når yaw-sticken er tilbake i senter så starter dronen
    throttle = motor_idle_speed;                                                   //Sett throttle lik 1200
    angle_pitch = angle_pitch_acc;                                                 //Vinkelen til gyroen(pitch) er lik vinkelen til akselerometeret
    angle_roll = angle_roll_acc;                                                   //Vinkelen til gyroen(roll) er lik vinkelen til akselerometeret
    ground_pressure = actual_pressure;                                             //Registrer trykket ved bakken
    course_lock_heading = angle_yaw;                                               //Retningen til dronen er course_lock_heading
    if (number_used_sats >= 5){
      lat_gps_home = l_lat_gps;
      lon_gps_home = l_lon_gps;
      home_point_recorded = 1;
    }
    else{
      home_point_recorded = 0;
    }
    start = 2;                                                                     //Sett start til 2 for å vise at den har startet
  }
/*

  if (start == 3 && channel_4 > 1350){                                               //Hva motorene gjør i modus 3
    angle_pitch = angle_pitch_acc;                                                 
    angle_roll = angle_roll_acc;                                                   
    ground_pressure = actual_pressure;                                           
    course_lock_heading = angle_yaw;                                             
    start = 2;
    resetPID();
  }
*/

  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;                                                                     //Sett start til 0 for å si at den har landet
    takeoff = 0;                                                                   //Si at dronen har landet
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  if (takeoff == 0 && start == 2) {

    if (channel_3 <= motor_idle_speed) {                                           //Reset PID før man tar av                                                                       
      resetPID();                                                                
    }
    else {
      throttle = channel_3; 
    }
    
    if (((acc_z_average_total/ 25)*9.81/4096) > 11.3) {                            //takeoff er registrert når dronen akselererer oppover.
      takeoff = 1;                                                                 //indiker at dronen har tatt av
      takeoff_throttle = throttle;
      if (throttle > 1750) {                                                               
        error = 7;                                                                 //Hvis den er for tung gi feil nr. 7
      }
    }
  }
}
