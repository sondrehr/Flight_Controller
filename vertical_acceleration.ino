
//Den vertikale akselerasjonen er kalkulert over en lengre periode via roterende array
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vertical_acceleration_calculations() {
  
  if (acc_z_average_mem_location == 25) {                                                //minnet har 25 plasser(0-24). holder telleren innefor det gyldige området
    acc_z_average_mem_location = 0;
  }

  acc_z_average_total -= acc_z_average[acc_z_average_mem_location];                      //Hindrer at totalen vokser i det uendelige
  acc_z_average[acc_z_average_mem_location] = acc_z;
  acc_z_average_total += acc_z_average[acc_z_average_mem_location];

  acc_z_average_mem_location++;
  
}
