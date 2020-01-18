
void start_stop_takeoff() {
  
  if (channel_3 < 1050 && channel_4 < 1050 && channel_2 > 1050 && start != 3) {      //starte modus 2
    start = 1;                                                                       //Starte motoren: nederst til venstre
  }

 /*                                                                                 
  if (channel_3 < 1050 && channel_4 < 1050 && channel_2 < 1050 && channel_1 < 1050){ //starte modus 3
    start = 3;                                                                       //For å ta auto-takeoff. Begge stikkene nederst til venstre
  }
*/
  
  if (start == 1 && channel_4 > 1350) {                                      //When yaw stick is back in the center position start the motors (step 2).
    throttle = motor_idle_speed;                                                   //Set the base throttle to the motor_idle_speed variable.
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                             //Register the pressure at ground level for altitude calculations.
    course_lock_heading = angle_yaw;                                               //Set the current compass heading as the course lock heading.
    if (number_used_sats >= 5){
      lat_gps_home = l_lat_gps;
      lon_gps_home = l_lon_gps;
      home_point_recorded = 1;
    }
    else{
      home_point_recorded = 0;
    }
    start = 2;                                                                     //Set the start variable to 2 to indicate that the quadcopter is started.
  }
/*

  if (start == 3 && channel_4 > 1350){                                               //Hva motorene gjør i modus 3
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                           //Register the pressure at ground level for altitude calculations.
    course_lock_heading = angle_yaw;                                             //Set the current compass heading as the course lock heading.
    start = 2;
    resetPID();
  }
*/

  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
    takeoff = 0;                                                          //Reset the auto take-off detection.
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  if (takeoff == 0 && start == 2) {

    if (channel_3 <= motor_idle_speed) {                                           //When the throttle is below the center stick position.                                                                       
      resetPID();                                                                  //When the throttle is back at idle speed reset the PID controllers.
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
