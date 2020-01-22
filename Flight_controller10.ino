#include <EEPROM.h>
#include <Wire.h>               

TwoWire HWire (2, I2C_FAST_MODE);
 
//LED
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t error, error_counter, error_led;
uint32_t error_timer;

uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint32_t flight_mode_timer;

float low_battery_warning = 10.5;


//PID
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
float pid_p_gain_roll = 2.0;               //Gain P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain I-controller (default = 0.04).
float pid_d_gain_roll = 20.0;              //Gain D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output fra PID

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output fra PID.

float pid_p_gain_yaw = 4.0;                //Gain P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output fra PID

float battery_compensation = 40.0;

float pid_error_temp;

float pid_i_mem_roll_angle, pid_i_mem_roll_rate, pid_output_roll, pid_roll_d_error_angle, pid_roll_d_error_rate;
float pid_i_mem_pitch_angle, pid_i_mem_pitch_rate, pid_output_pitch, pid_pitch_d_error_angle, pid_pitch_d_error_rate;
float pid_i_mem_yaw, pid_output_yaw, pid_yaw_d_error;

float pid_roll_setpoint, gyro_roll_input;
float pid_pitch_setpoint, gyro_pitch_input;
float pid_yaw_setpoint, gyro_yaw_input;

float roll_level_adjust, pitch_level_adjust;

//Return to home
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float return_to_home_decrease;
float angle;


//Telemetry_data
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t check_byte;

uint32_t telemetry_buffer_byte;

uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;

float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

//Tx_and_Rx
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t channel_7_start, channel_7;
int32_t channel_8_start, channel_8;
int32_t channel_9_start, channel_9;
int32_t channel_10_start, channel_10;

int32_t measured_time, measured_time_start;

uint8_t channel_select_counter;

uint16_t receiver_watchdog;


//calibration_baro_gps
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


float angle_roll_acc, angle_pitch_acc;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

uint8_t level_calibration_on;


//Gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t cal_int;

int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

int16_t acc_y, acc_x, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;


//Start_stop_takeoff
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t manual_throttle;
int16_t throttle, takeoff_throttle;

int16_t motor_idle_speed = 1100;

uint8_t start;

uint8_t takeoff;


//timer_setup
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t esc_1, esc_2, esc_3, esc_4;


//vertical_acceleration_calculations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t acc_z_average[25];

uint8_t acc_z_average_mem_location;

int32_t acc_total_vector, acc_total_vector_at_start;

int32_t acc_z_average_total;



//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t loop_timer;                                                          //For å holde styr på hvor lang loopen er

float battery_voltage;

int32_t pid_roll_setpoint_base;                                               //input for å endre roll
int32_t pid_pitch_setpoint_base;                                              //input for å endre pitch
int32_t pid_yaw_setpoint_base;

int i;                                                                        //brukes i alle for-looper
int j;

float angle_pitch, angle_roll, angle_yaw;

float angle_acc_pitch_comp;
float angle_acc_roll_comp;

float total_angle_pitch;
float total_angle_roll;

//Compass variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t compass_calibration_on, heading_lock;
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;

float inclination = 0.0;
float declination = 4.0;                                                      //Forskjellen mellom geografisk nord og magnetisk nord


//Pressure variables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t temperature;
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;


//Altitude PID variables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t manual_altitude_change;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

float pid_p_gain_altitude = 1.4;           //Gain altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Gain altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                ///Maximum output fra PID


//read_GPS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north, longitude_east;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
uint8_t home_point_recorded;
uint8_t return_to_home_step;
int32_t lat_gps_home, lon_gps_home;


float gps_p_gain = 2.7;                    //Gain GPS P-controller (default = 2.7).
float gps_d_gain = 8.0;                    //Gain GPS D-controller (default = 6.5).

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
 
  
  pinMode(4, INPUT_ANALOG);
  pinMode(PA11, OUTPUT);                                        //Grønn LED
  pinMode(PA12, OUTPUT);                                        //Rød LED
  pinMode(PC13, OUTPUT);                                        //LED brukes til GPS
  digitalWrite(PC13, HIGH);                                     //Motsatt funksjon (HØY = LAV)
  pinMode(PB0, OUTPUT);                                         //Set PB0 as output for telemetry TX

  green_led(LOW);
  red_led(HIGH);


  //Minne som bevares selv om den skrus av!!!!!
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;

  timer_setup();                                                //Sett opp timerene som brukes av esc og reciever
  delay(50);                                                    //La timerene starte opp

  gps_setup();                                                  //Sett opp GPS, baud rate osv...


  //Starter kommunikasjon med de ulike sensorene via I2C
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //MPU6050

  HWire.begin();                                                //start I2C
  HWire.beginTransmission(0x68);
  error = HWire.endTransmission();                              //Registrer hva som blir sendt. Hvis det ikke er 0 = feil
  while (error != 0) {
    error = 1;
    error_signal();
    delay(4);
  }

  //Kompass

  HWire.beginTransmission(0x1E);
  error = HWire.endTransmission();
  while (error != 0) {
    error = 2;
    error_signal();
    delay(4);
  }


  //MS5611

  HWire.beginTransmission(0x77);
  error = HWire.endTransmission();
  while (error != 0) {
    error = 3;
    error_signal();
    delay(4);
  }


  //Setter opp sensorene
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //HMC5883l
  ////////////////////////////////////////////////////////

  setup_compass();
  read_compass();

  angle_yaw = actual_compass_heading;                           //Sett den initielle kompass-headingen

  //MPU6050
  //////////////////////////////////////////////////////////

  gyro_setup();

  //5 sekunders forsinkelse
  for (i = 0; i < 1250; i++) {                                  //1250 * 4 micro = 5 s
    if (i % 125 == 0) {                                         //125 looper =  (500ms).
      digitalWrite(PA12, !digitalRead(PA12));                   //Endre LED
    }
    delay(4);                                                   //Simuler 250Hz refresh
  }

  calibrate_gyro();                                             //Kalibrer gyro offset


  //ms5611
  /////////////////////////////////////////////////////////

  for (start = 1; start <= 6; start++) {
    HWire.beginTransmission(0x77);                              //Start kommunikasjon
    HWire.write(0xA0 + start * 2);                              //Send addressen vi vil lese fra
    HWire.endTransmission();                                    //End "samtalen"

    HWire.requestFrom(0x77, 2);                                 //Hent 2 bytes
    C[start] = HWire.read() << 8 | HWire.read();
  }

  OFF_C2 = C[2] * pow(2, 16);
  SENS_C1 = C[1] * pow(2, 15);

  for (start = 0; start < 100; start++) {                      //Gi sensoren tid til å stabiliserer seg
    read_barometer();
    delay(4);
  }
  actual_pressure = 0;



  //Sjekker at batteriet + fjernkontrollen fungerer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 4;
    error_signal();
    delay(4);
  }

  error = 0;                                                       //Hvis den får signal fra fjernkontrollen sett feilen lik 0


  red_led(LOW);                                                    //Når alt ferdig, skru av rød LED



  //12 bit
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //spenningen som måles og den egentlige er i forholdet: 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //4095 / 36.3= 112.81.

  battery_voltage = (float)analogRead(4) / 112.81;



  for (i = 0; i <= 24; i++) {
    acc_z_average[i] = acc_z;
  }
  acc_z_average_total = acc_z * 25;                              //Den gjennomsnitlige verdien til akselerometeret blir lagret.



  start = 0;

  loop_timer = micros();                                         //Start timeren for loopen

  Serial.begin(57600);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

//Flymoduser
//////////////////////////////////////////////////////////////

  if (start == 0) {

    if (channel_1 > 1900 && channel_2 > 1900 && channel_3 > 1900 && channel_4 > 1900) {       //For å kalibrere kompasset ta begge stikkene øverst til høyre
      calibrate_compass();
    }

    if (channel_1 < 1100 && channel_2 > 1900 && channel_3 > 1900 && channel_4 < 1100) {       //For å kalibrere akselerometeret ta begge stikkene øverst til venstre
      calibrate_level();
    }
  }

  heading_lock = 0;
  if (channel_5 > 1200)heading_lock = 1;


  flight_mode = 1;                                                                 //Bestemmer flymodus
  if (channel_7 >= 1200 && channel_5 < 1600){
    flight_mode = 2;
  }
  if (channel_7 >= 1600 && channel_5 < 2100){
    flight_mode = 3;
    if (channel_8 > 1500){
      flight_mode = 4;
    }
  }


  if (flight_mode <= 3) {
    return_to_home_step = 0;
    return_to_home_lat_factor = 0;
    return_to_home_lon_factor = 0;
  }

  

  
  return_to_home();                                                                //fly hjem
  flight_mode_signal();                                                            //Vis flymodus
  error_signal();                                                                  //Vis feil


  //Leser alle de ulike sensorene
  //////////////////////////////////////////////////

  gyro_signal();                                                                   //Les data fra gyroen

  read_barometer();                                                                //Les data fra barometeret

  read_compass();                                                                  //Les data fra kompasset

  if (gps_add_counter >= 0) {
    gps_add_counter --;
  }
  read_gps();


  //Gyro utregninger
  //////////////////////////////////////////////////

  //utregningene som skal brukes i PID regulatoren

  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);        //Deler på 65.5 for å få i deg/sek
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);     //Bruker et komplementær filter for å minske støy
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);



  //integrer opp de ulike vinklene

  //dt = 0.004 = 1/250
  //verdien = gyro_x/65.5
  //integrer opp vinkelen: Vinkel += vinkelfart * dt = gyro_x/65.5 * 1/250
  //1/65.5 * 1/250 = 0.0000611

  angle_pitch += (float)gyro_pitch * 0.0000611;
  angle_roll += (float)gyro_roll * 0.0000611;
  angle_yaw += (float)gyro_yaw * 0.0000611;


  if (angle_yaw < 0) {
    angle_yaw += 360;                                                              //Hvis yaw er mindre enn 0 pluss på 360 for at verdien skal være gyldig
  }
  else if (angle_yaw >= 360) {
    angle_yaw -= 360;                                                              //Hvis yaw blir større enn 360 trekk fra 360 for at verdien skal være gyldig
  }


  //Hvis dronen er i en vinkel og det utføres yaw

  //0.000001066 = 0.0000611 * (3.142 / 180) = 1/250(dt) * 1/65.5(deg/sec) * (3.14 / 180)
  //Gjøres om til radianer siden sin funksjonen i arduino bruker radianer

  angle_pitch -= total_angle_roll * sin((float)gyro_yaw * 0.000001066);
  angle_roll += total_angle_pitch * sin((float)gyro_yaw * 0.000001066);


  angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Korigerer for drift i yaw aksen. Returnerer angle_yaw - actual_compass_heading


  if (angle_yaw < 0) {
    angle_yaw += 360;                                                              //Hvis yaw er mindre enn 0 pluss på 360 for at verdien skal være gyldig
  }
  else if (angle_yaw >= 360) {
    angle_yaw -= 360;                                                              //Hvis yaw blir større enn 360 trekk fra 360 for at verdien skal være gyldig
  }



  //Akselerometer-utregninger
  ////////////////////////////////////////////////////////////


  acc_total_vector = sqrt((acc_y * acc_y) + (acc_x * acc_x) + (acc_z * acc_z));    //Total akselerometer vektor


  if (abs(acc_x) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_x / acc_total_vector) * 57.296;              //Finner vinkelen som akselerometeret gir
  }


  if (abs(acc_y) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }


  vertical_acceleration_calculations();                                            //regner ut vertikal akselerasjon

  //Sensor fusjon
  ////////////////////////////////////////////////////////

  angle_pitch = (angle_pitch * 0.9996) + (angle_pitch_acc * 0.0004);
  angle_roll = (angle_roll * 0.9996) + (angle_roll_acc * 0.0004);                  //Bruker et komplementær-filter for å kompensere for drift i gyroen


  pitch_level_adjust = angle_pitch * 15;                                           //Bruker en enkel P kontroller for å gi et pådrag lik en konstant ganger vinkelen
  roll_level_adjust = angle_roll * 15;




  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  pid_roll_setpoint_base = channel_1;                                              //Normally channel_1 is the pid_roll_setpoint input.
  pid_pitch_setpoint_base = channel_2;                                             //Normally channel_2 is the pid_pitch_setpoint input.
  pid_yaw_setpoint_base = channel_4;
  gps_man_adjust_heading = angle_yaw;                                              //Brukes for å endre gps-waypoint i riktig retning hvis head-lock er aktivert

  //Head lock vil si at pitch og yaw ikke lenger er avhengig av retningen på dronen
  //Når dronen starter, lagrer den headingen i variabelen course_lock_heading
  //Først finner vi forskjellen mellom kursen vi startet i og kursen vi nå holder
  //Basert på denne forskjellen blir pitch og roll kalkulert
  
  if (heading_lock == 1) {
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    pid_roll_setpoint_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
    pid_pitch_setpoint_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    gps_man_adjust_heading = course_lock_heading;
  }


  //GPS hold
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (flight_mode >= 3 && waypoint_set == 1) {
    pid_roll_setpoint_base = 1500 + gps_roll_adjust;
    pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
  }
  
  //Passer på at alle verdier er innenfor gyldige områder
  if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  calculate_pid();                                                                 

  start_stop_takeoff();                                                            //Start, stopp og take-off detection

  //Må kompensere for fall i spenningen
  //Et komplementærfilter brukes til å redusere støy
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);

  //Skru på led hvis spenningen blir for lav.
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0) {
    error = 1;
  }


  //Regner ut basis for throttle som er "kraften" til hver motor
  //Er også her vi starter altitude hold
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////7

  if (takeoff == 1 && start == 2) {                                         //If the quadcopter is started and flying.
    throttle = channel_3;                                         //The base throttle is the receiver throttle channel + the detected take-off throttle.
    if (channel_3 < 1550 && channel_3 > 1700) {
      throttle = 1610;
    }
    if (flight_mode >= 2) {                                                          //If altitude mode is active.
      throttle = 1610 + pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
    }
  }
/*
  Serial.print(takeoff_throttle);
  Serial.print("\n");
*/
  //Her blir alt(PID + throttle + timere) slått sammen for å porodusere signalene til esc-ene
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (start == 2) {                                                                //starter motorene
    if (throttle > 1800) {
      throttle = 1800;
    }
    esc_1 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Puls for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Puls for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Puls for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Puls for esc 4 (front-left - CW)

    if (battery_voltage < 12.40 && battery_voltage > 6.0) {                        //kompenserer for spenningsfall
      esc_1 += (12.40 - battery_voltage) * battery_compensation;
      esc_2 += (12.40 - battery_voltage) * battery_compensation;
      esc_3 += (12.40 - battery_voltage) * battery_compensation;
      esc_4 += (12.40 - battery_voltage) * battery_compensation;
    }

    if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                        //Hvis motorene er under idle speed, sett fart lik idle speed
    if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;
    if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;
    if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Maks-puls lik 2000
    if (esc_2 > 2000)esc_2 = 2000;
    if (esc_3 > 2000)esc_3 = 2000;
    if (esc_4 > 2000)esc_4 = 2000;
  }

  else {
    esc_1 = 1000;                                                                  //Hvis start != 2, fart = 0
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }


  TIMER4_BASE->CCR1 = esc_1;                                                       //Sett lengden på pulsen lik det vi har regnet ut at den skal bli
  TIMER4_BASE->CCR2 = esc_2;
  TIMER4_BASE->CCR3 = esc_3;
  TIMER4_BASE->CCR4 = esc_4;
  TIMER4_BASE->CNT = 5000;                                                         //Verdien blir nullstilt av software ikke ARR

  //send_telemetry_data();                                                         //Send telemetri til "bakkestasjonen"


  if (micros() - loop_timer > 4050) {
    error = 2;                                                                     //Hvis loopen varer i mer enn 4000 mikrosekunder = feil
  }
  while (micros() - loop_timer < 4000);                                            //Vent til 4000 mikrosekunder har gått
  loop_timer = micros();
}
