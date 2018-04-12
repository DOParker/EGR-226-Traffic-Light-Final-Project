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

void Traffic_Light_pin_init (void);     //initialize signal LEDs
void Pedestrian_North_init (void);      //initialize Pedestrian crossing LEDs
void IR_Sense_Init (void);              //initialize the IR sensor
void TimerA2_Output_Init (void);        //initialize PWM for IR output
void T32_INT2_IRQHandler (void);        //Interrupt Handler for Timer 2.
void TA0_N_IRQHandler(void);            //Interrupt Handler for Timer A0
void PORT1_IRQHandler(void);            //Interrupt Handler for Port 1

void NorthGreen (void);                 //function of state north green
void NorthYellow (void);                //function of state north yellow
void NorthRed (void);                   //function of state north red
void EastYellow (void);                 //function of state east Yellow
void NorthEastRed (void);               //function of state both red
void NorthPedestrian (void);            //function of state north pedestrian
void NorthPedestrianStop (void);        //function of state north pedestrain stop


uint8_t flag = 0, North_pedestrian = 0,Transit_Vehicle_Flag = 0, Emer_Vehicle_Flag = 0;     //initialize variables
uint8_t previous_state = 0, state = NGreen, previous_flag = 0, nRedFlag = 0;                //initialize variables
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

void TA0_N_IRQHandler(void)
{
    if (TIMER_A0->CCTL[2] & BIT0)                           // T0.4 was the cause.  This is setup as a capture.  Successful capture means blink LED1.
    {
        currentedge = TIMER_A0->CCR[2];
        period = currentedge - lastedge;
        lastedge = currentedge;

        if ( ( 17817 <period ) && ( period < 19688 ) )          // within 5% of 10Hz period
        {
            Emer_Vehicle_Flag++;                                //set flag for which frequency detected
            Transit_Vehicle_Flag = 0;
        }

        else if ( ( 25446 < period ) && ( period < 28055 ) )    // within 5% of 14Hz period
        {
            Emer_Vehicle_Flag = 0;                              //set flag for which frequency detected
            Transit_Vehicle_Flag++;
        }
        else
        {
            Emer_Vehicle_Flag = 0;                              //if appropriate Hz not detected reset flags
            Transit_Vehicle_Flag = 0;
        }

        TIMER_A0->CCTL[2] &= ~(TIMER_A_CCTLN_CCIFG);            // Clear the interrupt flag
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
/***************************State Functions*************************************/
void NorthGreen (void)
{
    uint8_t previous_Emer_Flag = 0;         //declare local variables
    
    if (flag && Emer_Vehicle_Flag == 0)
    {
        state = NYellow;                //increment state
        TIMER32_2->LOAD = 9000000;      //3 sec count for NYellow state
        flag = 0;                       //reset flag
    }
    if (flag == 0 && Emer_Vehicle_Flag == 0)
    {
        P4->OUT |= BIT0;                //North green ON
        P4->OUT |= BIT5;                //East red ON
        P4->OUT &= ~0b01011110;         //All else OFF
    }
    if (flag == 0 && Emer_Vehicle_Flag)
    {
        if(Emer_Vehicle_Flag == 1)
            previous_state = state;

        if(Emer_Vehicle_Flag != previous_Emer_Flag)
        {
            TIIMER32_2->LOAD  = 3000000;                //load timer with 1 sec if interrupt happend again
            previous_Emer_Flag = Emer_Vehicle_Flag;     //reset previous to current
        }
        P4->OUT &= ~0b01011110;                         //make sure LEDs are in proper configuration
    }
    if (flag && Emer_Vehicle_Flag)
    {
        
    }
}

void NorthYellow (void)
{
    if (flag)
    {
        state = BothRed;                //increment state
        TIMER32_2->LOAD = 3000000;      //1 sec count for BothRed state
        flag = 0;                       //reset flag
        previous_state = NYellow;       //store current state for return in BothRed state
    }
    if (flag == 0 && Emer_Vehicle_Flag == 0)
    {
        P4->OUT |= BIT1;                //North yellow ON
        P4->OUT |= BIT5;                //East red ON
        P4->OUT &= ~0b01011101;         //All else OFF
    }
    if (flag == 0 && Emer_Vehicle_Flag)
    {
        if(Emer_Vehicle_Flag == 1)
            previous_state = state;

        state = NGreen;                 //send to next state to maintain order
        Emer_Vehicle_Flag++;            //make sure previous state does not get set again
        P4->OUT &= ~0b01011110;         //make sure LEDs are in proper configuration
    }
}

void NorthRed (void)
{
    if (flag)
    {
        state = EYellow;                //increment state
        TIMER32_2->LOAD = 9000000;      //3 sec count for EYellow
        flag = 0;                       //reset the flag
        nRedFlag = 0;
    }
    if(flag == 0 && Emer_Vehicle_Flag)
    {
        if(Emer_Vehicle_Flag)
            previous_state = state;

        state = EYellow;                //send state to appropriate state to maintain order
        Emer_Vehicle_Flag++;            //make sure previous state does not get set again
        TIMER32_2->LOAD = 90000000;     //timer set for 3 sec
    }
    if(flag == 0 && North_pedestrian == 0 && Emer_Vehicle_Flag == 0)
    {
        P4->OUT |= BIT2;                //North red ON
        P4->OUT |= BIT3;                //East green ON
        P4->OUT &= ~0b01110011;         //All else OFF
        nRedFlag = 1;
    }
    if (flag == 0 && North_pedestrian == 1 && nRedFlag == 0 && Emer_Vehicle_Flag == 0)
    {
        state = Npedestrian;
        TIMER32_2->LOAD = 45000000;
    }
}

void EastYellow (void)
{
    if (flag && Emer_Vehicle_Flag == 0)
    {
        state = BothRed;                //increment state
        TIMER32_2->LOAD = 3000000;      //1 sec count for BothRed state
        flag = 0;                       //reset flag
        previous_state = EYellow;       //store current state for return in BothRed state
    }
    else if (flag == 0 && Emer_Vehicle_Flag == 0)
    {
        P4->OUT |= BIT2;                //North red ON
        P4->OUT |= BIT4;                //East yellow ON
        P4->OUT &= ~0b01101011;         //All else OFF
    }
    else if(Emer_Vehicle_Flag == 1 && flag)
    {
        state = BothRed;
        TIMER32_2->LOAD = 30000000;
        flag = 0;
    }
}

void NorthEastRed (void)
{
    if (flag && Emer_Vehicle_Flag == 0)
    {
        state = (previous_state +1) % 4;        //increment state to next NRed/NGreen
        if (previous_state == NYellow)          //is next state NRed
            TIMER32_2->LOAD = 24000000;         //8 sec wait for NRed
        else if(previous_state == EYellow)      //is next state NGreen
            TIMER32_2->LOAD = 36000000;         //12 sec wait for NGreen
        flag = 0;
        previous_state = 0;
    }
    else if(flag && Emer_Vehicle_Flag)
    {
        state = NGreen;                         //send to correct state for order
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
        TIMER32_2->LOAD = 750000;
        flag = 0;
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
        North_pedestrian = 0;
        state = EYellow;
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

    TIMER32_2->CONTROL = 0b11100011;            //Sets timer 2 for Enabled,
                                                //Periodic
                                                //With Interrupt
                                                //No Prescaler, 32 bit mode
                                                //One Shot Mode.  See 589 / reference manual

    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);   // Enable Port 3 interrupt on the NVIC
    NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31);   // Enable Timer A0 interrupt on the NVIC
    NVIC_EnableIRQ( T32_INT2_IRQn );            //Enable Timer32_2 interrupt.
    __enable_interrupts ();                    //Enable all interrupts for MSP432

    TIMER32_2->LOAD = 36000000;                 //12 sec count for NGreen state/initial state
    P4->OUT |= BIT7;

    while (1)
    {
        switch (state)
        {
            case NGreen:
                NorthGreen ();                  //send to the north green function
                break;

            case NYellow:
                NorthYellow ();
                break;

            case NRed:
                NorthRed ();
                break;

            case EYellow:
               EastYellow ();
               break;

            case BothRed:
                NorthEastRed ();
                break;

            case Npedestrian:
                NorthPedestrian ();
                break;

            case Npedestrian_stop:
                NorthPedestrianStop ();
               break;
        }
        printf ("%d\n", Emer_Vehicle_Flag);
    }
}
