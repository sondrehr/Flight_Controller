
void timer_setup() {
  Timer2.attachCompare1Interrupt(handler_channel_1);                          //utfører handler_channel_1 når det blir interupted
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE;                                       //Gjør det mulig å attache en interrupt      
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;                            //Koble reciever input til timeren som brukes for å finne en rising edge
  TIMER2_BASE->CCMR2 = 0;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E;

  TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;                                      //Brukes for detektere enn stigende edge
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;                                                  //auto reload: Hvor lang tid timeren bruker på en runde
  TIMER2_BASE->DCR = 0;



//Output til ESC
/////////////////////////////////////////////////////////////////////////////////////////////

  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 5000;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = 1000;

  TIMER4_BASE->CCR1 = 1000;                                             //Output er høy til timeren kommer til <- antall mikrosekunder
  TIMER4_BASE->CCR2 = 1000;                                             //4 ulike timere som blir reset av programmet og ikke ARR
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  pinMode(PB6, PWM);                                                    //PB6 er standard output for timer 4 channel 1
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}
