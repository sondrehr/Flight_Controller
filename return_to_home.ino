void return_to_home() {

  if (flight_mode == 4) {

//Sjekk om dronen er nærmere enn 10 meter
//Regner ut blant annet høyden den må bevege seg til
/////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (return_to_home_step == 0) {

//Hvis dronen er mindre enn  meter unna. Kan den bare synke
/*
      if (abs(lat_gps_home - l_lat_waypoint) < 90 && abs(lon_gps_home - l_lon_waypoint) < 90){
        return_to_home_step = 4;
      }
*/


//Regn ut forholdet mellom lat_error og lon_error slik at den beveger seg i en rett linje mot målet
      
     /* else {*/
        /*
        return_to_home_move_factor = 0.0;
        if (return_to_home_lat_factor == 1 || return_to_home_lon_factor == 1){
          return_to_home_step = 1;
        }
        */


        if (abs(lat_gps_home - l_lat_waypoint) >= abs(lon_gps_home - l_lon_waypoint)) {
          return_to_home_lon_factor = (float)abs(lon_gps_home - l_lon_waypoint) / (float)abs(lat_gps_home - l_lat_waypoint);
          return_to_home_lat_factor = 1;
        }
        else {
          return_to_home_lon_factor = 1;
          return_to_home_lat_factor = (float)abs(lat_gps_home - l_lat_waypoint) / (float)abs(lon_gps_home - l_lon_waypoint);
        }

        return_to_home_step = 2;



//Regner ut hvor mye den må stige.  
//170 millibar er 20 meter. 17?
//Trekk fra høyden
//F.eks. 170 - (1000 - 1100) = 70

        if (ground_pressure - actual_pressure < 50/*170*/){
          return_to_home_decrease = 50/*170*/ - (ground_pressure - actual_pressure);      
        }
        else return_to_home_decrease = 0;
      /*}*/
    }


//Snur seg mot home
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    if (return_to_home_step == 1){
      angle = atan2((l_lon_waypoint - lon_gps_home), (l_lat_waypoint - lat_gps_home)) * 57.296;
      
      if (abs(angle_yaw - angle) < 10 || abs(angle_yaw - angle) > 350){
        return_to_home_step = 2;
      }

      else { 
        pid_yaw_setpoint_base = 20;
      }
    }
*/

//stig til 20 meter over bakken
/////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (return_to_home_step == 2) {
      if (return_to_home_decrease <= 0){
        return_to_home_step = 3;
      }
      if (return_to_home_decrease > 0) {
        pid_altitude_setpoint -= 0.035;
        return_to_home_decrease -= 0.035;
      }

      return_to_home_move_factor = 0.0;                                                                     //Må initialisere verdien slik at den ikke blir nullet ut hver gang
    }
   
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//Dronen styres av endingen av GPS-waypoint. Så når det punktet er lik Home -> steg 3
 
    if (return_to_home_step == 3) {
      if (lat_gps_home == l_lat_waypoint && lon_gps_home == l_lon_waypoint){                              
        return_to_home_step = 4;
      }


//Hvis avstanden er mindre enn  meter senk farten
//Hvis ikke, øk farten
      
      if (abs(lat_gps_home - l_lat_waypoint) < 160 && abs(lon_gps_home - l_lon_waypoint) < 160 && return_to_home_move_factor > 0.05){
        return_to_home_move_factor -= 0.00015;
      }
      else if (return_to_home_move_factor < 0.20){
        return_to_home_move_factor += 0.0001;
      }


      if (lat_gps_home > l_lat_waypoint){
        l_lat_gps_float_adjust += return_to_home_move_factor * return_to_home_lat_factor;
      }
      if (lat_gps_home < l_lat_waypoint){
        l_lat_gps_float_adjust -= return_to_home_move_factor * return_to_home_lat_factor;
      }



      if (lon_gps_home > l_lon_waypoint){
        l_lon_gps_float_adjust += return_to_home_move_factor * return_to_home_lon_factor;
      }
      if (lon_gps_home < l_lon_waypoint){
        l_lon_gps_float_adjust -= return_to_home_move_factor * return_to_home_lon_factor;
      }
        
/*
      if (lat_gps_home != l_lat_waypoint) {
        if (lat_gps_home > l_lat_waypoint){
          l_lat_gps_float_adjust += return_to_home_move_factor * return_to_home_lat_factor;
        }
        if (lat_gps_home < l_lat_waypoint){
          l_lat_gps_float_adjust -= return_to_home_move_factor * return_to_home_lat_factor;
        }
      }
      
      if (lon_gps_home != l_lon_waypoint) {
        if (lon_gps_home > l_lon_waypoint){
          l_lon_gps_float_adjust += return_to_home_move_factor * return_to_home_lon_factor;
        }
        if (lon_gps_home < l_lon_waypoint){
          l_lon_gps_float_adjust -= return_to_home_move_factor * return_to_home_lon_factor;
        }
      }
*/      
    }


    
//Når den har landet kan ikke actual_pressure stige mer så da vil forskjellen bli større
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    if (return_to_home_step == 4) {
      if (pid_altitude_setpoint > actual_pressure + 150){
        return_to_home_step = 5;
      }                            
      pid_altitude_setpoint += 0.035;
    }


    
//Stopper motorene
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (return_to_home_step == 5) {                                                                         
      start = 0;
      return_to_home_step = 6;
    }

  }
}
