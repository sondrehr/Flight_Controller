
//Oppløsning:

void read_compass() {
  HWire.beginTransmission(0x1E);                     
  HWire.write(0x03);                                            
  HWire.endTransmission();                                      

  HWire.requestFrom(0x1E, 6);                        
  compass_y = HWire.read() << 8 | HWire.read();                 
  compass_y *= -1;                                              
  compass_z = HWire.read() << 8 | HWire.read();                 
  compass_x = HWire.read() << 8 | HWire.read();                 
  compass_x *= -1;                                             //Finner de ulike aksene og inverter slik at vi får x og y retningen i rikitg orientasjon


  
  if (compass_calibration_on == 0) {                            
    compass_y += compass_offset_y;                              
    compass_y *= compass_scale_y;                               //Skalerer den så den matcher de andre aksene
    compass_z += compass_offset_z;                              //Legg til offset
    compass_z *= compass_scale_z;                               
    compass_x += compass_offset_x;                            
  }
  


 //Ferdige formler for x og y gitt fra datasheete til kompasset.
  compass_x_horizontal = compass_x * cos(angle_pitch * 0.0174533) + compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * 0.0174533) - compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * 0.0174533);
  compass_y_horizontal = compass_y * cos(angle_roll * 0.0174533) + compass_z * sin(angle_roll * 0.0174533);
  
  actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);
  
  actual_compass_heading -= declination;                                 //Legg til declination slik at den peke mot geografisk nord og ikke magnetisk nord
  
  if (actual_compass_heading < 0){
    actual_compass_heading += 360;         
  }
  else if (actual_compass_heading >= 360) {
    actual_compass_heading -= 360;                                       //Holder den innenfor 360 grader
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup_compass() {
  HWire.beginTransmission(0x1E);                   
  HWire.write(0x00);                                           
  HWire.write(0x78);                                            
  HWire.write(0x20);                                           
  HWire.write(0x00);                                            //initialiserer de ulike registrene
  HWire.endTransmission();                                   

 
  for (j = 0; j < 6; j ++){
    compass_cal_values[j] = EEPROM.read(0x10 + j);
  } 

  
  //Formler for å kalibrere kompasset og for å legge til offsett som oppført i databladet til kompasset
  
  compass_scale_y = (compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = (compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = ((compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = ((compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180){
      base_course_mirrored = course_c - 180;              //Flipper verdiene slik at alle er innenfor et gyldig område
    }
    else {
      base_course_mirrored = course_c + 180;
    }

    
    if (course_b > 180){
      actual_course_mirrored = course_b - 180;
    }
    else {
      actual_course_mirrored = course_b + 180;
    }
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}
