// AUTHORS: NICHOLAS JENS, STEVE DIAZ, KARUN DHILLON
// EEC 195 
// MKL25Z4 BOARD - FREESCALE CUP TRACK CODE
// TEAM WEITANG CLAN (2016)
#include <MKL25Z4.h>
#include "uart.h"
#include "adc16.h"
#include "timers.h"
#include "main.h"       
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

volatile uint32_t msTicks;
void init_ADC0(void);
// INITIALIZE PING PONG BUFFER
int prevservopulse = 0;
int state = 0, prevstate = 0, prevprevstate = 0;
int rflag = 0, lflag = 0;

int trackmid = 0;
int buffer1[128];
int buffer2[128];
int line[128];
int line2[128];
int mph = 72;						//GLOBAL SPEED SET BUFFER // last one = 75
char str[3], newline[2] = "\r\n";
int buffselect = 0;					//buffer selection flag
int k = 0;							//index variable
int delayer = 0, flag = 0;				
int j = 0; 							//print index count

int clkcount = 0;
int val1 = 0;
int clearflag = 0;

volatile uint8_t switch1;
volatile uint8_t switch2;
volatile uint8_t buffer;
unsigned int sw2flag, sw1flag;

int location1 = 0, location2 = 0;

volatile unsigned int PW1 = 4500, PW2 = 0, PW3 = 0;
volatile unsigned int pulse1, pulse2;
int count = 0;						//TPM counter
volatile char TPMflag = 0;
unsigned int steeringlflag = 5;          // TPM control flag


void Init_ADC(void) {
	
	init_ADC0();			// initialize and calibrate ADC0
	ADC0->CFG1 = (ADLPC_LOW | ADIV_1 | ADLSMP_LONG | MODE_8 | ADICLK_BUS_2);	// 8 bit, Bus clock/2 = 12 MHz
	ADC0->SC2 = 0;		// ADTRG=0 (software trigger mode)
	
	/* Enable Interrupts */
	NVIC_SetPriority(ADC0_IRQn, 64); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);
}


unsigned int Poll_ADC (void) {
	volatile unsigned int res = 0;	
	
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		
									// wait for conversion to complete (polling)

	res = ADC0->R[0];				// read result register
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	return res;
}
void TPM1_IRQHandler(void) {
//clear pending IRQ
	NVIC_ClearPendingIRQ(TPM1_IRQn);
	
// clear the overflow mask by writing 1 to TOF
	
	if (TPM1->SC & TPM_SC_TOF_MASK) 
		TPM1->SC |= TPM_SC_TOF_MASK;

// modify pulse width for TPM1_CH1
	
	if (steeringlflag) {
            TPM1->CONTROLS[0].CnV = PW1;            
	}		
}


void GPIOinit() {
	  SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK
									| SIM_SCGC5_PORTB_MASK
									| SIM_SCGC5_PORTC_MASK
									| SIM_SCGC5_PORTD_MASK
									| SIM_SCGC5_PORTE_MASK );
	  SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; // set PLLFLLSEL to select the PLL for this clock source
	  SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);    // select the PLLFLLCLK as UART0 clock source
	    
	  PORTA->PCR[1] = PORT_PCR_MUX(0x2);		// Enable the UART0_RX function on PTA1
		
	  PORTA->PCR[2] = PORT_PCR_MUX(0x2);		// Enable the UART0_TX function on PTA2
	    
	  PORTB->PCR[2] = PORT_PCR_MUX(0);				// PTB2 = analog input (adc0_se12)
	  PORTB->PCR[3] = PORT_PCR_MUX(0);				// PTB3 = analog input (adc0_se13)
	  //FPTB->PDDR |= (1UL << 2)|(1UL << 3);  
	  PORTC->PCR[13] = (1UL <<  8);			// SW1 (PTC13) is gpio
	  PORTC->PCR[17] = (1UL <<  8);	        // SW2 (PTC17) is gpio            
	  
	  PORTC->PCR[3] = (1UL << 8);             // HBRIDGE PTC3 is gpio
	  PORTC->PCR[4] = (1UL << 8);             // HBRIDGE PTC4 is gpio
	  FPTC->PDDR=1UL<<4;
      
	  PORTC->PCR[1] = (1UL << 8);             // HBRIDGE PTC1 is gpio
	  PORTC->PCR[2] = (1UL << 8);             // HBRIDGE PTC2 is gpio
	  FPTC->PDDR=1UL<<2;  

	  PORTE->PCR[21] = (1UL << 8);			// Pin PTE21 (HBRIDGE-CONTROL) is GPIO
	  FPTE->PDDR |= (1UL << 21);
	  FPTE->PDOR = 1UL << 21;				    // PTE21 (HBRIDGE) set LOW
	  //CAMERA STUFF*__*/PIT STUFF
	  PORTB->PCR[8] = (1UL << 8);				// Pin PTB8 is GPIO
	  FPTD->PDDR |= (1UL << 8);					// PTB8 out (signal to measure conversion time)
	  
	  
	  PORTB->PCR[0] = (1UL << 8); 				// Pin PTB0 is GPIO 
	  FPTB->PDOR |= 1;							// initialize PTB0
	  FPTB->PDDR |= 1;							// configure PTB0 as output
	  
	  //other stuff
	  PORTD->PCR[7] = (1UL << 8);			// Pin PTD7 (SI) is GPIO
	  FPTD->PDDR |= (1UL << 7);				// PTD7 out
		
		
	  PORTD->PCR[5] = (1UL << 8);			// Pin PTD5 (AO) is GPIO
	  
	  PORTE->PCR[1] = (1UL << 8);			// Pin PTE1 (CLK) is GPIO
	  FPTE->PDDR |= (1UL << 1);				// PTE1 out
	  
	   
    
}

//ADC HANDLER (FILL CAMERA BUFFER)
void ADC0_IRQHandler(void){
  FPTE->PSOR = 1UL << 1;	//set (CLK) PTE1 to high
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  val1 = ADC0->R[0];				//read and store into val1
  clkcount++;
	
  if (buffselect == 0)  {
    buffer1[k] = val1;  //store value
	k++;				//incrememnt index
    
	if (clkcount < 129){
		if(delayer == false)
			delayer--;
		ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6);		// start conversion on channel SE6b (PTD5)
    }		
	else {				
		flag = 1;		//if counter < 129 set flag true
		FPTB->PCOR = 1UL << 8;
	}
  }
  
  if(buffselect == 1){
	buffer2[k] = val1;
	k++;

    if(clkcount < 129){
			if(delayer == true)
				delayer++;
		ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6);
	
	}	
	else{
		flag = 1;
		FPTB->PCOR = 1UL << 8;
	}
  }

  FPTE->PCOR = 1UL << 1;			//deassert CLK  signal
}  //ADCInterruptHandler()

void PIT_IRQHandler(void){
  //LEDGreen_On();
  int s;
  FPTD->PSOR = 1UL << 7;		//PTD7 to high
  FPTB->PSOR = 1UL << 8;		//PTB8 to high	"assert a gpio signal for measuring conversion time"
  clkcount = 0;

  if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK)  {		//clear status flag for timer channel 0
    PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;
  }  
  
  if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK)  {		//clear status flag for timer channel 1
    PIT->CHANNEL[1].TFLG &= PIT_TFLG_TIF_MASK;
  }  
  
  delayer++;
  FPTE->PSOR = 1UL << 1;  						           		//assert clk signal
  clkcount = 1;
  k = 0;				//index = 1 
  
  //switch buffers
  if (buffselect == 0)
	  buffselect = 1;
  else if (buffselect == 1)
	  buffselect = 0;
  
  delayer--;
  FPTD->PCOR = 1UL << 7;								//deassert SI signal
  
  
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;    				// select b channel 
 
  ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6); // start conversion on channel SE6b (PTD5)
  
  FPTE->PCOR = 1UL << 1;								//deassert clk signal
}//PIT HANDLER


void put(char *ptr_str){
	while(*ptr_str)
		uart0_putchar(*ptr_str++);
}
void Init_PWM(void) {

// Set up the clock source for MCGPLLCLK/2. 
// See p. 124 and 195-196 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
// TPM clock will be 48.0 MHz if CLOCK_SETUP is 1 in system_MKL25Z4.c.
	
	SIM-> SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	//SIM-> SOPT2 |= (SIM_SOPT2_TPMSRC(0) | SIM_SOPT2_PLLFLLSEL_MASK);
	
// See p. 207 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
	SIM->SCGC6 |= (SIM_SCGC6_TPM1_MASK)|(SIM_SCGC6_TPM0_MASK); // Turn on clock to TPM1


// See p. 163 and p. 183-184 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
	PORTB->PCR[0] = PORT_PCR_MUX(3); // Configure PTB0 as TPM1_CH0
    PORTC->PCR[1] = PORT_PCR_MUX(4); // Configure PTC3 as TPM0_CH0
	PORTC->PCR[3] = PORT_PCR_MUX(4); // Configure PTC1 as TPM0_CH2
	
// Set channel TPM1_CH1 to edge-aligned, high-true PWM
	
	TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	
// Set period and pulse widths
	
	TPM1->MOD = 60000-1;		// Freq. = (48 MHz / 16) / 3000 = 1 kHz
	TPM0->MOD = 600-1;   
	TPM1->CONTROLS[0].CnV = PW1;
	TPM0->CONTROLS[0].CnV = PW2; 	
	TPM0->CONTROLS[2].CnV = PW3; 	 	
	
// set TPM1 to up-counter, divide by 16 prescaler and clock mode
	
	TPM1->SC = (TPM_SC_CMOD(1) | TPM_SC_PS(4));
	TPM0->SC = (TPM_SC_CMOD(1) | TPM_SC_PS(4));
	
// clear the overflow mask by writing 1 to TOF
	
	if (TPM1->SC & TPM_SC_TOF_MASK) TPM1->SC |= TPM_SC_TOF_MASK;
	if (TPM0->SC & TPM_SC_TOF_MASK) TPM0->SC |= TPM_SC_TOF_MASK;

// Enable Interrupts

	NVIC_SetPriority(TPM1_IRQn, 192); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM1_IRQn); 
	NVIC_EnableIRQ(TPM1_IRQn);	
	NVIC_SetPriority(TPM0_IRQn, 192); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM0_IRQn); 
	NVIC_EnableIRQ(TPM0_IRQn);

}	


// FUNCTION - emergency protocol for undetected course
void unclear(int prevst, int prevprevst){
  int leftwheel, rightwheel;
  if (prevst == 2 && prevprevst == 2)
  {
    state = 2;
    rightwheel = ((600*mph*11)/10 >> 8);
    TPM0->CONTROLS[2].CnV = rightwheel;
	leftwheel = ((600*mph)/2 >> 8);
	TPM0->CONTROLS[0].CnV = leftwheel;
  }
  else if(prevst == 4 && prevprevst == 4)
  {
    state = 4;
    rightwheel = ((600*mph)/2 >> 8);
	TPM0->CONTROLS[2].CnV = rightwheel;
	leftwheel = ((600*mph*11)/10 >> 8);
	TPM0->CONTROLS[0].CnV = leftwheel;
  }

}

// FUNCTION to control the steering servo + motors based off camera line buffer
void steeringcontrol(int tracktop, int trackbot, int line[128])
{
    // state: 0 = initial/straight, 1 = right, 2 = hard right, 3 = left, 4 = hard left
    char str[] = "\r\errorcontrol\r\n";  //FOR TESTING
	int ctrl,p, i,error_R=0,error_L=0,rightwheel,leftwheel,hitL = 0,hitR = 0;
	int leftturn = 0, rightturn = 0, lefthardturn = 0, righthardturn = 0, sumR = 0, sumRR =0, sumLL = 0, sumL = 0, sumM = 0;
	int uncleartrack = 0;
    double mp = .70, np = .85, kp, zp = 1.1,xp = 0.2;
    //calculate control variables - ERROR_R ERROR_L
    error_R = tracktop - 5;
	error_L = 123 - trackbot;    
    //calculate possible turns PWs 

    for(i = 0; i < 128; i++){if(line[i] == 1) sumM = sumM + 1;}
    if (sumM > 80) {uncleartrack = 1;}
    
	//pulse1 = 5500 - (1000*(58 - error_R))/58;	
	//pulse2 = 3500 + (1000*(58 - error_L))/58;    
    
	for ( p = 0; p < 64; p++) {if(line[p] == 1) hitL = hitL + 1;}
    for ( p = 65; p < 128; p++) {if(line[p] == 1) hitR = hitR + 1;}
    ctrl = hitL - hitR;
    
   if (uncleartrack == 1)
   {
     state = prevstate;
     TPM1->CONTROLS[0].CnV = prevservopulse;
     TPM0->CONTROLS[2].CnV = (600*(mph) >> 8) * xp;
     TPM0->CONTROLS[0].CnV = (600*(mph) >> 8) * xp;
   }
   else if(uncleartrack == 0)
   {    //LEFT
        if(ctrl > 10){
          kp = 1250*abs(ctrl)/64;
          if (kp < 120) // if straight
          {
            state = 0;
            kp = 0;     // dont turn
            mp = 1.2;   // double motor speed
            TPM0->CONTROLS[2].CnV = (600*(mph) >> 8) *mp;
            TPM0->CONTROLS[0].CnV = (600*(mph) >> 8) *mp; 
          }
          if(kp > 500)
          {
            state = 2;
            mp = .3;
            TPM0->CONTROLS[0].CnV = (600*(mph) >> 8) *mp; 	// Put pot values into PWM
            TPM0->CONTROLS[2].CnV = (600*(mph) >> 8) *np; // speed up outter wheel  
          }
           //TPM0->CONTROLS[0].CnV = (600*(mph) >> 8) *mp;
           pulse1 = 4500 + kp;
           TPM1->CONTROLS[0].CnV = pulse1;
           prevservopulse = pulse1;
        }
        //RIGHT
        else if (ctrl < 10)
        {
          kp = 1250*abs(ctrl)/64;
          if(kp < 120)
          {
            state = 0;
            kp = 0;
            mp = 1.2;
            TPM0->CONTROLS[2].CnV = (600*(mph) >> 8) *mp;
            TPM0->CONTROLS[0].CnV = (600*(mph) >> 8) *mp; 
          }
          if (kp > 500)
          {
            state = 4;
            mp = .3;
            TPM0->CONTROLS[2].CnV = (600*(mph) >> 8) * mp; 	// Put pot values into PWM
            TPM0->CONTROLS[0].CnV = (600*(mph) >> 8) * np;  
          }
          //TPM0->CONTROLS[2].CnV = (600*(mph) >> 8) *mp; 
          pulse2 = 4500 - kp;
          TPM1->CONTROLS[0].CnV = pulse2;
          prevservopulse = pulse2;
        }
    } //uncleartrack == 0
   
    prevstate = state;
    prevprevstate = prevstate;
    
	//FOR TESTING PURPOSES
	//sprintf(str,"err-R:%d err-L:%d %d %d",error_R,error_L, pulse1, pulse2);put(str);
}

// MAIN FUNCTION - initialize peripherals, infinite main control loop
int main (void) {
	int tracktop, trackbot, trackcount = 0, trackset = 0, trackset2 = 0, topset = 0, botset = 0, trackcount2 = 0;
	int printer[128];
    unsigned int val = 0, val2;
    char key, key2, prompt[36] = "Enter 'o' to continue or 'q' to quit";
	int uart0_clk_khz,  min = 100000000, max = 0, i, sum = 0, avg, p2p, printcount = 0;
	char str[] = "\r\n<<<TURN ON POWER SUPPLY, THEN PRESS (SW2)>>> \r\n";
    char str2[] = "\r\nSET DUTY CYCLES USING POT1 THEN PRESS (SW1) ~ jk inactive\r\n";
	//A. PERIPHERAL INITIALIZATION
	SystemCoreClockUpdate();
    GPIOinit();
	uart0_clk_khz = (48000000 / 1000); // UART0 clock frequency will equal half the PLL frequency	
	uart0_init (uart0_clk_khz, TERMINAL_BAUD); 
    Init_ADC();
    Init_PWM();
	Init_PIT(7000);							// 100Hz-----set as 1000= 1ms or 10000	= 10ms
	Start_PIT();
    for (i = 0; i < 128; i++){buffer1[i] = 0; buffer2[i] = 0; line[i] = 0; line2[i] = 0;}//initialize buffers
    
   
    //B. PRINT MESSAGE
	put(str);
    // C. WAIT FOR SW2/ D. ENABLE HBRIDGE
    while (1) {
	  switch2 = (PTC->PDIR>>17)& 0x01;
      if(switch2 == 0x01) 
      {
        put("\r\n<<<MOTOR IS RUNNING>>>\r\n");
		FPTE->PSOR = 1UL << 21;
		break;
      }
    }
    // E. PRINT MESSAGE 2
    put(str2);
    // F. SETTINGS
    while (1) {	  
	  switch1 = (PTC->PDIR>>13)& 0x01;		//store the value of sw1 pushed on or off


      ADC0->SC1[0] = 0xD;//DIFF_SINGLE|ADC_SC1_ADCH(13);      // start conversion AD13 (POT1)
      val = Poll_ADC();

      PW2 = 600*mph >> 8;  // set duty cycle PWM birdge A from POT 1 val
      
	  TPM0->CONTROLS[0].CnV = PW2; 	//set right side speed
	  TPM0->CONTROLS[2].CnV = PW2;  //set left side speed

      if(switch1 == 0x01) { break;}
     }//while


    //MAIN WHILE LOOP
    while(1)  
    {	
      if (uart0_getchar_present())  {
	    key = uart0_getchar();
		if ( key == 'p' ) { 
		    printcount = 0;
			if(buffselect == 0){  put("\r\nPing Buffer: \r\n"); for(i = 0; i < 129; i++){sprintf(str,"%d",line[i]); put(str); put(" ");}}
			else if(buffselect == 1){  put("\r\nPong Buffer: \r\n"); for(i = 0; i < 129; i++){sprintf(str,"%d",line[i]); put(str); put(" ");}}
            put(prompt);
            key2 = '0';
            if (key2 == '0') {
              for(;;){
                key = uart0_getchar();
                    
                if(key == 'o' || key == 'q'){
                    if (key == 'o') {
                      key2 = '1';
                      break;
                    }
                    if(key == 'q')  {
                      Stop_PIT();
                      put("\r\nPROGRAM EXIT\r\n");
                      exit(0);	
                    }
                }
                    
              }
           }			            
		}
	  }	  
	  else if (!(uart0_getchar_present())) {
		if (buffselect == 0) {
                for (i = 5; i < 123; i++){
                  if(buffer1[i] < min)  {min = buffer1[i];}		//set the minimum		          
                  if(buffer1[i] > max)  {max = buffer1[i];}		//set the maximum		    
                  sum += buffer1[i];
                }  // calculate buffer data(max/min/sum) to calculate threshhold value
                p2p = max-min;
                avg = sum/128 - 5;  //threshhold 
                
				//CALCULATE TRACK METHOD
                //put("\r\n");
                for (i = 0; i < 10; i++) {line[i] = 0;}
                for (i = 10; i < 118; i++)  {
                    if (buffer1[i] < avg) {line[i] = 1;}
                    else if (buffer1[i] > avg) {line[i] = 0;}
                }	
                for(i = 118; i < 128; i++) {line[i] = 0;}
				
				for (i = 58; i > 4; i--){
					if(topset == 0 && ((line[i] + line[i-1] + line[i-2] + line[i-3] +line[i-4] + line[i-5] + line[i-6]) == 4))
					{topset = 1; tracktop = i -2;}
				}
				if(topset == 0){tracktop = 5;}
				for (i = 70; i < 123; i++)	{
					if(botset == 0 && ((line[i] + line[i+1] + line[i+2] + line[i+3] + line[i+4] + line[i+5] + line[i+6]) == 4))
					{botset = 1; trackbot= i +2;}
				}
				if(botset == 0){trackbot = 123;}

				//FOR TRACK PRINT TESTING
                //put("\r\n");
                //for(i = 0; i < 128; i++)
				//{
				//  if (line[i] == 0) {put("-");}
				//  if (line[i] == 1) {put("x");}
				//  if (line[i] == 2) {put("U");}
				//}   


                steeringcontrol(tracktop, trackbot,line);


                trackcount = 0;
				trackset = 0;
				trackset2 = 0;
                trackbot = 0;
                tracktop = 0;
				botset = 0;
				topset = 0;
                max = 0;
                min = 100000000;  //reset for next buffer data calculation
                sum = 0; 
		}//buffselect==0
		if (buffselect == 1) {
                for (i = 5; i < 123; i++){
                  if(buffer2[i] < min)  {min = buffer2[i];}		//set the minimum		          
                  if(buffer2[i] > max)  {max = buffer2[i];}		//set the maximum		    
                  sum += buffer2[i];
                }  // calculate buffer data(max/min/sum) to calculate threshhold value
                p2p = max-min;
                avg = sum/128 - 5;  //threshhold 
                
                //put("\r\n");
                for (i = 0; i < 10; i++) {line[i] = 0;}
                for (i = 10; i < 118; i++)  {
                    if (buffer1[i] < avg) {line[i] = 1;}
                    else if (buffer1[i] > avg) {line[i] = 0;}
                }	
                for(i = 118; i < 128; i++) {line[i] = 0;}
				
				for (i = 63; i > 4; i--){
					if(topset == 0 && ((line[i] + line[i-1] + line[i-2] + line[i-3] +line[i-4] + line[i-5]+ line[i-6]) == 4))
					{topset = 1; tracktop = i -2;}
				}
				if(topset == 0){tracktop = 5;}
				for (i = 65; i < 123; i++)	{
					if(botset == 0 && ((line[i] + line[i+1] + line[i+2] + line[i+3] + line[i+4] + line[i+5]+ line[i+6]) == 4))
					{botset = 1; trackbot= i +2;}
				}
				if(botset == 0){trackbot = 123;}

                //FOR PRINT TESTING
				//put("\r\n");
                //for(i = 0; i < 128; i++)
				//{
				//  if (line[i] == 0) {put("-");}
				//  if (line[i] == 1) {put("x");}
				//  if (line[i] == 2) {put("U");}
				//}   


                steeringcontrol(tracktop, trackbot, line);


                trackcount = 0;
				trackset = 0;
				trackset2 = 0;
                trackbot = 0;
                tracktop = 0;
				botset = 0;
				topset = 0;
                max = 0;
                min = 100000000;  //reset for next buffer data calculation
                sum = 0; 
		}//buffselect==1
		  
      }  
	}  //while(1)
           
 }//main()
