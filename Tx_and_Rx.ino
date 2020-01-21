
void handler_channel_1() {
  measured_time = TIMER2_BASE->CCR1 - measured_time_start;        //Forskjellen på forrige rising edge og denne
  if (measured_time < 0) {
    measured_time += 0xFFFF;                                      //Brukes hvis det er negativ tid som kan skyldes at registeret har nådd enden
  }
  measured_time_start = TIMER2_BASE->CCR1;                        //Reseter tellingen når alle kanalene har blitt sendt for å gjøre den klar til neste "batch" med kanaler
  if (measured_time > 3000) {
    channel_select_counter = 0;
  }
  else channel_select_counter++;

  if (channel_select_counter == 1)channel_1 = measured_time;
  if (channel_select_counter == 2)channel_2 = measured_time;
  if (channel_select_counter == 3)channel_3 = measured_time;
  if (channel_select_counter == 4)channel_4 = measured_time;
  if (channel_select_counter == 5)channel_5 = measured_time;
  if (channel_select_counter == 6)channel_6 = measured_time;
  if (channel_select_counter == 7)channel_7 = measured_time;
  if (channel_select_counter == 8)channel_8 = measured_time;
  if (channel_select_counter == 9)channel_9 = measured_time;
  if (channel_select_counter == 10)channel_10 = measured_time;
}

void print_kanaler() {
  Serial.print(channel_1);
  Serial.print("\t");
  Serial.print(channel_2);
  Serial.print("\t");
  Serial.print(channel_3);
  Serial.print("\t");
  Serial.print(channel_4);
  Serial.print("\t");
  Serial.print(channel_5);
  Serial.print("\t");
  Serial.print(channel_6);
  Serial.print("\t");
  Serial.print(channel_7);
  Serial.print("\t");
  Serial.print(channel_8);
  Serial.print("\t");
  Serial.print(channel_9);
  Serial.print("\t");
  Serial.print(channel_10);
  Serial.print("\n");
}
