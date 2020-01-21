
void calculate_pid() {

//roll Rx input
/////////////////////////////////////////////////  
                                                                                     
  pid_roll_setpoint = 0;
                                                                                        //siden signalene fra Tx varierer er det nødvendig med en liten dødsone
  if (pid_roll_setpoint_base > 1508){                                                   //setter setpoint lik 0 hvis den er innenfor dødsonen
    pid_roll_setpoint = (pid_roll_setpoint_base - 1508);                           //for å få rundt 30 grader pitch setpoint
  }
  else if (pid_roll_setpoint_base < 1492){
    pid_roll_setpoint = (pid_roll_setpoint_base - 1492);
  }
  
  
  pid_roll_setpoint -= roll_level_adjust;                                          //Bruker en enkel P-regulator for å trekke fra vinkelen ganger en konstant Kp slik at dronen holder seg stabil selv om den ikk får noe input
  pid_roll_setpoint /= 3.0;                                                        //(500-8)/3 = 164d/s 
 

//pitch Rx input
////////////////////////////////////////////////
                                                                                       
  pid_pitch_setpoint = 0;
                                                                                        //siden signalene fra Tx varierer er det nødvendig med en liten dødsone
  if (pid_pitch_setpoint_base > 1508){                                                  //setter setpoint lik 0 hvis den er innenfor dødsonen
    pid_pitch_setpoint = (pid_pitch_setpoint_base - 1508);
  }
  else if (pid_pitch_setpoint_base < 1492){
    pid_pitch_setpoint = (pid_pitch_setpoint_base - 1492);
  }


  pid_pitch_setpoint -= pitch_level_adjust;                                        //Bruker en enkel P-regulator for å trekke fra vinkelen ganger en konstant Kp slik at dronen holder seg stabil selv om den ikk får noe input
  pid_pitch_setpoint /= 3.0;                                                       //(500-8)/3 = 164d/s 


//Yaw Rx input
////////////////////////////////////////////////

  pid_yaw_setpoint = 0;
                                                                                        
  if (channel_3 > 1050) {                                                               //Unødvendig med yaw når motorene er av
    if (channel_4 > 1508){
      pid_yaw_setpoint = (pid_yaw_setpoint_base - 1508) / 3.0;                                        //siden signalene fra Tx varierer er det nødvendig med en liten dødsone
    }                                                                                   //setter setpoint lik 0 hvis den er innenfor dødsonen
    else if (channel_4 < 1492){         
      pid_yaw_setpoint = (pid_yaw_setpoint_base - 1492) / 3.0;                                        //Bruker vinkelfart på yaw siden det gir mer mening enn vinkel: (500-8)/3 = 164d/s 
    }
  }


//Roll calculations
/////////////////////////////////////////////////////
  
  pid_error_temp = pid_roll_setpoint - gyro_roll_input;

  pid_i_mem_roll_rate += pid_error_temp * pid_i_gain_roll;                                             
  
  if (pid_i_mem_roll_rate > pid_max_roll){
    pid_i_mem_roll_rate = pid_max_roll;                                                      //anti wind-up
  }
  else if (pid_i_mem_roll_rate < pid_max_roll * -1){
    pid_i_mem_roll_rate = pid_max_roll * -1;
  }

  pid_output_roll = ((pid_p_gain_roll * pid_error_temp) + pid_i_mem_roll_rate + (pid_d_gain_roll * (pid_error_temp - pid_roll_d_error_rate)));
/*
  Serial.print(pid_i_mem_roll_rate);
  Serial.print("\t");
*/
  
  if (pid_output_roll > pid_max_roll){                                                 //begrensning i pådrag                
    pid_output_roll = pid_max_roll;                                       
  }
  else if (pid_output_roll < pid_max_roll * -1){
    pid_output_roll = pid_max_roll * -1;
  }

  pid_roll_d_error_rate = pid_error_temp;



  
//Pitch calculations
/////////////////////////////////////////////////////

  pid_error_temp = pid_pitch_setpoint - gyro_pitch_input;
  
  pid_i_mem_pitch_rate += pid_error_temp * pid_i_gain_pitch;
  
  if (pid_i_mem_pitch_rate > pid_max_pitch){
    pid_i_mem_pitch_rate = pid_max_pitch;
  }
  else if (pid_i_mem_pitch_rate < pid_max_pitch * -1){
    pid_i_mem_pitch_rate = pid_max_pitch * -1;
  }

  pid_output_pitch = ((pid_p_gain_pitch * pid_error_temp) + pid_i_mem_pitch_rate + (pid_d_gain_pitch * (pid_error_temp - pid_pitch_d_error_rate)));


  
  if (pid_output_pitch > pid_max_pitch){
    pid_output_pitch = pid_max_pitch;
  }
  else if (pid_output_pitch < pid_max_pitch * -1){
    pid_output_pitch = pid_max_pitch * -1;
  }

  pid_pitch_d_error_rate = pid_error_temp;



//Yaw calculations
/////////////////////////////////////////////////////

  pid_error_temp = pid_yaw_setpoint - gyro_yaw_input;
  
  pid_i_mem_yaw += pid_error_temp * pid_i_gain_yaw;
  
  if (pid_i_mem_yaw > pid_max_yaw){
    pid_i_mem_yaw = pid_max_yaw;
  }
  else if (pid_i_mem_yaw < pid_max_yaw * -1){
    pid_i_mem_yaw = pid_max_yaw * -1;
  }

  pid_output_yaw = (pid_p_gain_yaw * pid_error_temp) + pid_i_mem_yaw + (pid_d_gain_yaw * (pid_error_temp - pid_yaw_d_error));

  
  if (pid_output_yaw > pid_max_yaw){
    pid_output_yaw = pid_max_yaw;
  }
  else if (pid_output_yaw < pid_max_yaw * -1){
    pid_output_yaw = pid_max_yaw * -1;
  }

  pid_yaw_d_error = pid_error_temp;

  /*
  Serial.print(angle_yaw);
  Serial.print("\t");
  Serial.print(actual_compass_heading);
  Serial.print("\n");
*/


}



void resetPID(){
  pid_i_mem_roll_angle = 0;
  pid_i_mem_roll_rate = 0;
  pid_output_roll = 0;
  
  pid_i_mem_pitch_angle = 0; 
  pid_i_mem_pitch_rate = 0;
  pid_output_pitch = 0;
  
  pid_i_mem_yaw = 0;
  pid_yaw_d_error = 0;
  pid_output_yaw = 0;
}
