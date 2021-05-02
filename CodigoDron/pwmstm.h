








void PWM_init(uint16_t frec){
 
 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN; // clock al puerto C

GPIOA->MODER |=  GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1 ; // varios modos
GPIOA->OTYPER &= ~( GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6) ; // Ensure push pull mode selected--default
GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR6); //Ensure all pull up pull down resistors are disabled
GPIOA->AFR[0] |= 0b00100010000000000000000000000000;


GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ; // varios modos
GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_1 | GPIO_OTYPER_OT_0) ; // Ensure push pull mode selected--default
GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR0); //Ensure all pull up pull down resistors are disabled
GPIOB->AFR[0] |= 0b00100010;


 
 /////////////////////Setear Timer/////////////////
 TIM3->CR1 &= ~ TIM_CR1_CKD; //Reset clock Division bit field
 TIM3->CR1 |= TIM_CLOCKDIVISION_DIV1 | TIM_CR1_URS; // 1 2 4/* Select DIV1 as clock division*/
 TIM3->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);/* Reset mode selection bit fields */

 TIM3->PSC = 0;//  fclab/(frec*0xffff)-1; 45mhz    fpwm=90mhz/(Arr*(psc+1))
 TIM3->ARR = 21000; // Auto reload value 600000

/////////////////////Setear PWMs/////////////////
 TIM3->EGR |= TIM_EGR_CC1G ;//generar evento en CC
 TIM3->EGR |= TIM_EGR_CC2G ;//generar evento en CC
 TIM3->EGR |= TIM_EGR_CC3G ;//generar evento en CC
 TIM3->EGR |= TIM_EGR_CC4G ;//generar evento en CC



 TIM3->CCMR1 |= TIM_CCMR1_OC1M;//Reset the Output Compare Mode Bits PWM2
 TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
 TIM3->CCMR1 |= TIM_CCMR1_OC2M;//Reset the Output Compare Mode Bits PWM2
 TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
 
 TIM3->CCMR2 |= TIM_CCMR2_OC3M;//Reset the Output Compare Mode Bits PWM2
 TIM3->CCMR2 &= ~TIM_CCMR2_CC3S;
 TIM3->CCMR2 |= TIM_CCMR2_OC4M;//Reset the Output Compare Mode Bits PWM2
 TIM3->CCMR2 &= ~TIM_CCMR2_CC4S;
 

 TIM3->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;//Polaridad negativa y activar
 TIM3->CCER |= TIM_CCER_CC2P | TIM_CCER_CC2E;//Polaridad negativa y activar
 TIM3->CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E;//Polaridad negativa y activar
 TIM3->CCER |= TIM_CCER_CC4P | TIM_CCER_CC4E;//Polaridad negativa y activar

 
 
 
 uint16_t ini=11000;
 
 TIM3->CCR1 = ini;
 TIM3->CCR2 = ini;
 TIM3->CCR3 = ini;
 TIM3->CCR4 = ini;


 
 TIM3->CR1 = TIM_CR1_CEN; // Enable timer
  
  //Serial.println(TIM3->PSC);

  
}







void armar(){
  Serial.println("Armando motores");
  uint16_t cero=11000;
 
 GPIOE->ODR |= (1<<1);
 
 delay(500);
 
 TIM3->CCR1 = cero;
 TIM3->CCR2 = cero;
 TIM3->CCR3 = cero;
 TIM3->CCR4 = cero;
 delay(1000);
  
}

void calibrar(){
  Serial.println("Calibrando");
  uint16_t maxx=20000;
  uint16_t minn=11000;
  
 GPIOE->ODR &= ~(1<<1);
 
 
 Serial.println("maximo");
 TIM3->CCR1 = maxx;
 TIM3->CCR2 = maxx;
 TIM3->CCR3 = maxx;
 TIM3->CCR4 = maxx;
 
 delay(500);
 GPIOE->ODR |= (1<<1);



 delay(5000);
 Serial.println("minimo");
 TIM3->CCR1 = minn;
 TIM3->CCR2 = minn;
 TIM3->CCR3 = minn;
 TIM3->CCR4 = minn;
 delay(8000);
 Serial.println("terminado");
}


uint16_t consta=11000;
uint16_t mult=3;

void m0(uint16_t val){
val=val*mult+consta;
TIM3->CCR4 = val ;
}


void m1(uint16_t val){
val=val*mult+consta;
TIM3->CCR3 = val ;
}


void m2(uint16_t val){
val=val*mult+consta;
TIM3->CCR2 = val ;
}


void m3(uint16_t val){
val=val*mult+consta;
TIM3->CCR1 = val ;
}
