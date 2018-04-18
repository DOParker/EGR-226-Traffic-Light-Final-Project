#include "msp.h"
#include "stdio.h"

#define NGreen 0
#define NYellow 1
#define NRed 2
#define EYellow 3
#define BothRed 4
#define Npedestrian 5
#define Npedestrian_stop 6
#define Emer_Vehicle 7

unsigned int notesHz[] = {10, 1, 3};
unsigned short interval[] = {3, 12, 5};

void Traffic_Light_pin_init (void);     //initialize signal LEDs
void Pedestrian_North_init (void);      //initialize Pedestrian crossing LEDs
void IR_Sense_Init (void);              //initialize the IR sensor
void TimerA2_Output_Init (void);        //initialize PWM for IR output
void TIMER_A3_init (void);              //initialize timer A3 for interrupt
void T32_INT2_IRQHandler (void);        //Interrupt Handler for Timer32_2.
void TA0_N_IRQHandler(void);            //Interrupt Handler for Timer A0
void TA3_0_IRQHandler(void);            //INterrupt handler for timerA3
void PORT1_IRQHandler(void);            //Interrupt Handler for Port 1

void NorthGreen (void);                 //function of state north green
void NorthYellow (void);                //function of state north yellow
void NorthRed (void);                   //function of state north red
void EastYellow (void);                 //function of state east Yellow
void NorthEastRed (void);               //function of state both red
void NorthPedestrian (void);            //function of state north pedestrian
void NorthPedestrianStop (void);        //function of state north pedestrain stop


uint8_t flag = 0, North_pedestrian = 0,Transit_Vehicle_Flag = 0, Emer_Vehicle_Flag = 0, LCD_Update_flag = 0;            //initialize variables
uint8_t previous_state = 0, state = NGreen, previous_flag = 0, nRedFlag = 0, LCD_Update_Flag = 0, new_state_flag = 0;   //initialize variables
int currentedge, lastedge =0, period;

/*********FUNCTIONS*************/
void Traffic_Light_pin_init (void)
{
    P4->SEL0 = 0;       //initialize port 4 for LEDs
    P4->SEL1 = 0;       //green to red => green to red (4.0->4.5)
    P4->DIR |= 0xFF;
    P4->OUT &= ~0xFF;   //amber 4.7, white 4.6
}

void Pedestrian_North_init (void)
{
    P3->SEL0 &= ~BIT2;
    P3->SEL1 &= ~BIT2;
    P3->DIR &= ~BIT2;
    P3->REN |= BIT2;                    // Enable Internal  resistor
    P3->OUT |= BIT2;                    // Enable pull-up resistor (P6.0 output high)
    P3->IES = BIT2;                     //Set pin interrupt to trigger when it goes from high to low
    P3->IE = BIT2;                      // Enable interrupt for P6.0
    P3->IFG = 0;                        // Clear all P6 interrupt flags
}

void IR_Sense_Init (void)                       //P2.5 IR input (2.4 output)
{
    P2->SEL0 |= BIT5;                           // TA0.CCI2A input capture pin, second function
    P2->SEL1 &= ~BIT5;                          // TA0.CCI2A input capture pin, second function
    P2->DIR  &= ~ BIT5;

    TIMER_A0->CTL |= TIMER_A_CTL_TASSEL_2 |     // Use SMCLK as clock source,
            TIMER_A_CTL_MC__UP |                // Start timer in UP mode
            TIMER_A_CTL_ID_3 |                  // divide by 8  375KHz to capture 10 & 14 Hz
            TIMER_A_CTL_CLR;                    // clear TA0R

    TIMER_A0->CCTL[2] =TIMER_A_CCTLN_CM_1 |     // Capture rising edge,
            TIMER_A_CCTLN_CCIS_0 |              // Use CCI2A=ACLK,
            TIMER_A_CCTLN_CCIE |                // Enable capture interrupt
            TIMER_A_CCTLN_CAP |                 // Enable capture mode,
            TIMER_A_CCTLN_SCS;                  // Synchronous capture

    TIMER_A0->CCR[0] = 65535;                   //set period for IR sensor as max
}

void Buzzer_Init (void)
{  
    P3->DIR |= BIT7; //change to buzzer pin
    P3->OUT &= ~BIT7;
}

void LCD_displayInfo (int state){
    //if statements for different states with for loops to increment amount of seconds for a specific state
}


void Sound_Play(unsigned freq_in_hz, unsigned duration_ms)
 {
     uint32_t i = 0;
     //uint32_t load = 0;
     float time_period_seconds = (1.0/freq_in_hz)*1000.0;
     //load = time_period_ms * 0.333;
     for(i=0;i<duration_ms;i++){
         P3->OUT |= BIT7; //change to pin of buzzer
         Systick_ms_delay(time_period_ms);
         P3->OUT &= ~BIT7;
         Systick_ms_delay(time_period_ms);
     }
 }

void TimerA2_Output_Init (void)
{
     P6->DIR |= BIT6;                               // Setup P6.6 as an TimerA0 controlled output, SEL = 01
     P6->SEL0 |= BIT6;
     P6->SEL1 &= ~(BIT6);

     TIMER_A2->CCR[0]  = 37500;                     // PWM Period (# cycles of clock)
     TIMER_A2->CTL = 0b0000001011010100;            // SMCLK, Divide by 8, Count Up, Clear to start
     TIMER_A2->CCTL[3] = 0b0000000011100100;        // CCR1 reset/set mode 7, without interrupt.
     TIMER_A2->CCR[3] = 18750;                      // Duty cycle to 50% to start (1/2 of CCR0).
}

void TIMER_A3_init (void)
{
  TIMER_A3->CCR[0] = 46875;                     //count for 1 sec
  TIMER_A3->CTL = TIMER_A_CTL_SSEL__SMCLK |     // SMCLK
                  TIMER_A_CTL_MC__CONTINUOUS |  // Continuous Mode   MC_2
                  TIMER_A_CTL_ID_3;             //divide source by 8

  TIMER_A3->EX0 = 0b111;                        //divide by 8 again

  TIMER_A3->CCTL[0] = TIMER_A_CCTLN_CCIE;       // TACCR0 interrupt enabled

}

void T32_INT2_IRQHandler (void)         //Interrupt Handler for Timer 2.
{
    TIMER32_2->INTCLR = 1;              //Clear interrupt flag so it does not interrupt again immediately.
    flag++;                             //increment flag
}

void PORT3_IRQHandler(void)             // Port1 ISR
{
    if ( P3->IFG & BIT2 )               // If P1.1 had an interrupt (going from high to low
        North_pedestrian = 1;
    P3->IFG &= ~BIT2;                   // Reset the interrupt flag
}

/*void PORT3_IRQHandler(void){
    if (P3->IFG & BIT2)
        if(!(P3->IES |= BIT7)){ //change to pin of buzzer 
            T32LOAD1 = 9000000;
            North_pedestrian = 1;
        }
        else{
            if(T32VALUE1 == 0)
                for(k = 0; k<25; k++)
                {
                  Sound_Play(8*notes[k], 100*interval[k]);
                }
        }
    P3->IES ^= BIT7; //change to pin of buzzer
}*/
            
void TA0_N_IRQHandler(void)
{
    if (TIMER_A0->CCTL[2] & BIT0)                           // T0.4 was the cause.  This is setup as a capture.  Successful capture means blink LED1.
    {
        currentedge = TIMER_A0->CCR[2];
        period = currentedge - lastedge;
        lastedge = currentedge;

        if ( ( 35625 <period ) && ( period < 39375 ) )          // within 5% of 10Hz period
        {
            Emer_Vehicle_Flag++;                                //set flag for which frequency detected
            Transit_Vehicle_Flag = 0;
        }

        if ( ( 25446 < period ) && ( period < 28055 ) )    // within 5% of 14Hz period
        {
            Emer_Vehicle_Flag = 0;                              //set flag for which frequency detected
            Transit_Vehicle_Flag++;
        }

        TIMER_A0->CCTL[2] &= ~(TIMER_A_CCTLN_CCIFG);            // Clear the interrupt flag
    }
}

void TA3_0_IRQHandler(void)
{
    LCD_Update_flag++;                              //increment flag
    TIMER_A3->CCTL[0] &= ~ TIMER_A_CCTLN_CCIFG;     // Clear the flag in CCTL[0]
}

void Systick_init (void)
{
    SysTick->CTRL = 0;              // disable SysTick During step
    SysTick->LOAD = 0x00FFFFFF;     // max reload value
    SysTick->VAL = 0;               // any write to current clears it
    SysTick->CTRL = 0x00000005;     // enable systic, 3MHz, No Interrupts
}

void Systick_ms_delay (uint32_t delay)
{
    SysTick -> LOAD = ((delay * 3000) - 1);                     //delay for 1 msecond per delay value
    SysTick -> VAL  = 0;                                        // any write to CVR clears it
    while ( (SysTick -> CTRL  &  0x00010000) == 0);             // wait for flag to be SET
}

void Systick_us_delay (uint32_t delay)
{
    SysTick -> LOAD = ((delay * 3) - 1);                     //delay for 1 usecond per delay value
    SysTick -> VAL  = 0;                                     // any write to CVR clears it
    while ( (SysTick -> CTRL  &  0x00010000) == 0);          // wait for flag to be SET
}


/***************************State Functions*************************************/
void NorthGreen (void)
{
    uint8_t previous_Emer_Flag;         //declare local variables

    if (flag && Emer_Vehicle_Flag == 0)             //if (1,0)
    {
        state = NYellow;                //increment state
        new_state_flag = 1;             //flag for new state set
        TIMER32_2->LOAD = 9000000;      //3 sec count for NYellow state
        flag = 0;                       //reset flag
    }
    if (flag == 0 && Emer_Vehicle_Flag == 0)        //if (0,0)
    {
        P4->OUT |= BIT0;                //North green ON
        P4->OUT |= BIT5;                //East red ON
        P4->OUT &= ~0b01011110;         //All else OFF
    }
    if (flag == 0 && Emer_Vehicle_Flag)             //if (0,1)
    {
        P4->OUT |= BIT0;                    //turn on proper LEDs
        P4->OUT |= BIT5;
        nRedFlag = 0;                       //clear light green when pedestrian hit it

        if(Emer_Vehicle_Flag == 1)
            previous_state = state;

        if(Emer_Vehicle_Flag != previous_Emer_Flag)
        {
            TIMER32_2->LOAD  = 3000000;                 //load timer with 1 sec if interrupt happend again
            previous_Emer_Flag = Emer_Vehicle_Flag;     //reset previous to current
        }

        if(Emer_Vehicle_Flag = previous_Emer_Flag)
            P4->OUT &= ~0b01011110;                     //make sure LEDs are in proper configuration
    }
    if (flag && Emer_Vehicle_Flag)                  //if (1,1)
    {
        TIMER32_2->LOAD = 15000000;         //load timer with 5 sec for wait time after emer vehicle passed through
        Emer_Vehicle_Flag = 0;
        previous_Emer_Flag = 0;
        flag = 0;
    }
}

void NorthYellow (void)
{
    if (flag)                           //if (1,0) OR (1,1)
    {
        state = BothRed;                //increment state
        new_state_flag = 1;             //flag for new state set
        TIMER32_2->LOAD = 3000000;      //1 sec count for BothRed state
        flag = 0;                       //reset flag
        previous_state = NYellow;       //store current state for return in BothRed state
    }
    if (flag == 0 && Emer_Vehicle_Flag == 0)            //if (0,0)
    {
        P4->OUT |= BIT1;                //North yellow ON
        P4->OUT |= BIT5;                //East red ON
        P4->OUT &= ~0b01011101;         //All else OFF
    }
    if (flag == 0 && Emer_Vehicle_Flag == 1)            //if (0,1) only in case of flag = 1
    {
        previous_state = state;

        state = NGreen;                 //send to next state to maintain order
        new_state_flag = 1;             //flag for new state set
        Emer_Vehicle_Flag++;            //make sure previous state does not get set again

        P4->OUT &= ~0b01011110;         //make sure LEDs are in proper configuration
    }
    if(flag == 0 && Emer_Vehicle_Flag > 1)              //if (0,1) in all other cases
    {
        P4->OUT &= ~0b01011101;         //LEDs in correct state
    }
}

void NorthRed (void)
{
    if (flag == 1 && North_pedestrian == 0)
    {
        state = EYellow;                //increment state
        new_state_flag = 1;             //flag for new state set
        TIMER32_2->LOAD = 9000000;      //3 sec count for EYellow
        flag = 0;                       //reset the flag
    }
    if(flag == 0 && Emer_Vehicle_Flag)
    {
        if(Emer_Vehicle_Flag)
            previous_state = state;

        state = EYellow;                        //send state to appropriate state to maintain order
        new_state_flag = 1;                     //flag for new state set
        Emer_Vehicle_Flag++;                    //make sure previous state does not get set again
        TIMER32_2->LOAD = 9000000;              //timer set for 3 sec
    }
    if(flag == 0 && Emer_Vehicle_Flag == 0)
    {
        P4->OUT |= BIT2;                //North red ON
        P4->OUT |= BIT3;                //East green ON
        P4->OUT &= ~0b01110011;         //All else OFF

        if (North_pedestrian == 0)
            nRedFlag = 1;
    }

    if (flag == 1 && North_pedestrian == 1 && nRedFlag == 0 && Emer_Vehicle_Flag == 0)
    {
        state = Npedestrian;
        new_state_flag = 1;             //flag for new state set
        TIMER32_2->LOAD = 45000000;
    }
}

void EastYellow (void)
{
    if (flag && Emer_Vehicle_Flag == 0)
    {
        state = BothRed;                //increment state
        new_state_flag = 1;             //flag for new state set
        TIMER32_2->LOAD = 3000000;      //1 sec count for BothRed state
        flag = 0;                       //reset flag
        previous_state = EYellow;       //store current state for return in BothRed state
    }
    if (flag == 0 && Emer_Vehicle_Flag == 0)
    {
        P4->OUT |= BIT2;                //North red ON
        P4->OUT |= BIT4;                //East yellow ON
        P4->OUT &= ~0b01101011;         //All else OFF
        nRedFlag = 0;
    }
    if(Emer_Vehicle_Flag && flag == 0)
    {
        P4->OUT |= BIT2;
        P4->OUT |= BIT4;

        if(Emer_Vehicle_Flag == 1)
            previous_state = state;

        P4->OUT &= ~0b01101011;
    }
    if(Emer_Vehicle_Flag && flag)
    {
        state = BothRed;                    //send to correct state for order
        new_state_flag = 1;                 //flag for new state set
        TIMER32_2->LOAD = 3000000;          //load timer with 1 sec for state both red
        flag = 0;                           //reset flag
    }
}

void NorthEastRed (void)
{
    if (flag && Emer_Vehicle_Flag == 0)
    {
        state = (previous_state +1) % 4;                                //increment state to next NRed/NGreen
        if (previous_state == NYellow && North_pedestrian == 0)         //is next state NRed
            TIMER32_2->LOAD = 24000000;                                 //8 sec wait for NRed
        if(previous_state == EYellow)                                   //is next state NGreen
            TIMER32_2->LOAD = 36000000;                                 //12 sec wait for NGreen
        if(previous_state == NYellow && North_pedestrian == 1)
            TIMER32_2->LOAD == 15000000;                                //5 sec wait for NRed before pedestrain walk
        new_state_flag = 1;                                             //flag for new state set
        flag = 0;
        previous_state = 0;
    }
    else if(flag && Emer_Vehicle_Flag)
    {
        state = NGreen;                         //send to correct state for order
        new_state_flag = 1;                     //flag for new state set
        flag = 0;                               //reset flag
        TIMER32_2->LOAD = 90000000;             //3 sec wait
    }
    else if (flag == 0)
    {
        P4->OUT |= BIT2;                    //North red ON
        P4->OUT |= BIT5;                    //East red ON
        P4->OUT &= ~0b01011011;             //All else OFF
    }
}

void NorthPedestrian (void)
{
    if(flag == 0)
    {
        P4->OUT |= BIT2;                    //North red ON
        P4->OUT |= BIT3;                    //East green ON
        P4->OUT |= BIT6;                    //Pedestrian light ON
        P4->OUT &= ~0b10110011;             //All else OFF
    }
    else
    {
        state = Npedestrian_stop;
        new_state_flag = 1;                 //flag for new state set
        TIMER32_2->LOAD = 750000;
        flag = 0;
        North_pedestrian = 0;
    }
}

void NorthPedestrianStop (void)
{
    P4->OUT &= ~BIT6;                   //pedestrian walk OFF

    if((flag != previous_flag)&& flag < 20){
        P4->OUT ^= BIT7;
        TIMER32_2->LOAD = 750000;
        previous_flag++;
    }
    else if(flag == 20){
        flag = 0;
        previous_flag = 0;
        P4->OUT |= BIT7;                     //turn pedestrian warning ON
        TIMER32_2->LOAD = 9000000;           //3 sec count for EYellow
        state = EYellow;
        new_state_flag = 1;                 //flag for new state set
    }
}

/********************************************END State Functions***************************************/
/**
 * main.c
 */

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                                 // stop watchdog timer

    Traffic_Light_pin_init ();                  //initialize MSP pins for Traffic Light
    Pedestrian_North_init ();                   //initialize pin 6.0 for button interrupt
    IR_Sense_Init ();                           //initialize the IR sensor
    Pedestrian_North_init ();                   //initialize the button for crosswalk
    TimerA2_Output_Init ();                     //initialize the IR sensor with PWM
    TIMER_A3_init ();                           //initialize timer A3 for LCD interrupt

    TIMER32_2->CONTROL = 0b11100011;            //Sets timer 2 for Enabled,
                                                //Periodic
                                                //With Interrupt
                                                //No Prescaler, 32 bit mode
                                                //One Shot Mode.  See 589 / reference manual

    TIMER32_1->CONTROL = 0b11100011;            //Sets timer 1 for Enabled,
                                                //Periodic
                                                //With No Interrupts
                                                //No Prescaler, 32 bit mode
                                                //One Shot Mode.  See 589 / reference manual

    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);   // Enable Port 3 interrupt on the NVIC
    NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31);   // Enable Timer A0 interrupt on the NVIC
    NVIC_EnableIRQ( T32_INT1_IRQn );            //Enable Timer32_1 interrupt.
    NVIC_EnableIRQ( T32_INT2_IRQn );            //Enable Timer32_2 interrupt.
    __enable_interrupts ();                     //Enable all interrupts for MSP432

    TIMER32_1->LOAD = 3000000;                  //one second count for timer 1
    TIMER32_2->LOAD = 36000000;                 //12 sec count for NGreen state/initial state
    P4->OUT |= BIT7;

    while (1)
    {
        switch (state)
        {
            case NGreen:
                NorthGreen ();                  //send to the north green function
                //LCD_displayInfo(NGreen);
                break;

            case NYellow:
                NorthYellow ();
                //LCD_displayInfo(NYellow);
                break;

            case NRed:
                NorthRed ();
                //LCD_displayInfo(NRed);
                break;

            case EYellow:
               EastYellow ();
               //LCD_displayInfo(EYellow);
               break;

            case BothRed:
                NorthEastRed ();
                //LCD_displayInfo(BothRed);
                break;

            case Npedestrian:
                NorthPedestrian ();
                //LCD_displayInfo(Npedestrian);
                break;

            case Npedestrian_stop:
                NorthPedestrianStop ();
                //LCD_displayInfo(Npedestrian_stop);
               break;
        }

        if (LCD_Update_Flag == 1)
        {
            if (new_state_flag == 1){
                
            //if state changed
                //print new state for north then 0 seconds in state
                //print new state for east then 0 seconds in state
                //reset new_state_flag
                //USE SWITCH CASE FOR WHICH STATE TO PRINT
            //if state not changed
                //send cursor to position and print update flag
        }
    }
}
