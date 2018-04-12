#include "msp.h"

#define NGreen 0
#define NYellow 1
#define NRed 2
#define EYellow 3
#define BothRed 4
#define Npedestrian 5
#define Npedestrian_stop 6

uint8_t flag = 0, North_pedestrian = 0;

void Traffic_Light_pin_init (void);
void T32_INT2_IRQHandler (void);        //Interrupt Handler for Timer 2.
void Pedestrian_North_init (void);
void PORT1_IRQHandler(void);

/*********FUNCTIONS*************/
void Traffic_Light_pin_init (void)
{
    P4->SEL0 = 0;
    P4->SEL1 = 0;
    P4->DIR |= 0xFF;
    P4->OUT &= ~0xFF; //amber 4.7, white 4.6
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

void T32_INT2_IRQHandler (void)     //Interrupt Handler for Timer 2.
{
    TIMER32_2->INTCLR = 1;          //Clear interrupt flag so it does not interrupt again immediately.
    flag++;                         //increment flag
}

void PORT3_IRQHandler(void)             // Port1 ISR
{
    if ( P3->IFG & BIT2 )               // If P1.1 had an interrupt (going from high to low
        North_pedestrian = 1;
    P3->IFG &= ~BIT2;                   // Reset the interrupt flag
}

/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    int previous_state = 0, state = NGreen, previous_flag = 0, nRedFlag =0;     //initialize variables

    Traffic_Light_pin_init ();                  //initialize MSP pins for Traffic Light
    Pedestrian_North_init ();                   //initialize pin 6.0 for button interrupt

    TIMER32_2->CONTROL = 0b11100011;            //Sets timer 2 for Enabled,
                                                //Periodic
                                                //With Interrupt
                                                //No Prescaler, 32 bit mode
                                                //One Shot Mode.  See 589 / reference manual

    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);   // Enable Port 6 interrupt on the NVIC
    NVIC_EnableIRQ( T32_INT2_IRQn );            //Enable Timer32_2 interrupt.
    __enable_interrupts ( );                    //Enable all interrupts for MSP432

    TIMER32_2->LOAD = 36000000;                 //12 sec count for NGreen state/initial state
    P4->OUT |= BIT7;

    while (1)
    {
        switch (state)
        {
            case NGreen:
                if (flag)
                {
                    state = NYellow;                //increment state
                    TIMER32_2->LOAD = 9000000;      //3 sec count for NYellow state
                    flag = 0;                       //reset flag
                }
                else
                {
                    P4->OUT |= BIT0;                //North green ON
                    P4->OUT |= BIT5;                //East red ON
                    P4->OUT &= ~0b01011110;         //All else OFF
                }
                break;

            case NYellow:
                if (flag)
                {
                    state = BothRed;                //increment state
                    TIMER32_2->LOAD = 3000000;      //1 sec count for BothRed state
                    flag = 0;                       //reset flag
                    previous_state = NYellow;       //store current state for return in BothRed state
                }
                else
                {
                    P4->OUT |= BIT1;                //North yellow ON
                    P4->OUT |= BIT5;                //East red ON
                    P4->OUT &= ~0b01011101;         //All else OFF
                }
                break;

            case NRed:
                if (flag)
                {
                    state = EYellow;                //increment state
                    TIMER32_2->LOAD = 9000000;      //3 sec count for EYellow
                    flag = 0;                       //reset the flag
                    nRedFlag = 0;
                }
                else if(flag == 0 && North_pedestrian == 0 )
                {
                    P4->OUT |= BIT2;                //North red ON
                    P4->OUT |= BIT3;                //East green ON
                    P4->OUT &= ~0b01110011;         //All else OFF
                    nRedFlag = 1;
                }
                else if (flag == 0 && North_pedestrian == 1 && nRedFlag == 0)
                {
                    state = Npedestrian;
                    TIMER32_2->LOAD = 45000000;
                }

                break;

            case EYellow:
                if (flag)
                {
                    state = BothRed;                //increment state
                    TIMER32_2->LOAD = 3000000;      //1 sec count for BothRed state
                    flag = 0;                       //reset flag
                    previous_state = EYellow;       //store current state for return in BothRed state
                }
                else
                {
                    P4->OUT |= BIT2;                //North red ON
                    P4->OUT |= BIT4;                //East yellow ON
                    P4->OUT &= ~0b01101011;         //All else OFF
                }
                break;

            case BothRed:
                if (flag)
                {
                    state = (previous_state +1) % 4;          //increment state to next NRed/NGreen
                    if (previous_state == NYellow)      //is next state NRed
                        TIMER32_2->LOAD = 24000000;     //8 sec wait for NRed
                    else if(previous_state == EYellow)  //is next state NGreen
                        TIMER32_2->LOAD = 36000000;     //12 sec wait for NGreen
                    flag = 0;
                    previous_state = 0;
                }
                else
                {
                    P4->OUT |= BIT2;                    //North red ON
                    P4->OUT |= BIT5;                    //East red ON
                    P4->OUT &= ~0b01011011;             //All else OFF
                }
                break;

            case Npedestrian:
                if(flag == 0){
                    P4->OUT |= BIT2;                    //North red ON
                    P4->OUT |= BIT3;                    //East green ON
                    P4->OUT |= BIT6;                    //Pedestrian light ON
                    P4->OUT &= ~0b10110011;             //All else OFF
                }
                else{
                    state = Npedestrian_stop;
                    TIMER32_2->LOAD = 750000;
                    flag = 0;
                }
                break;

            case Npedestrian_stop:
                P4->OUT &= ~BIT6;                   //pedestrian walk OFF

                /*SysTick->CTRL = 0x07;               //enable SysTick
                while (state == Npedestrian_stop);  //wait for interrupt
                SysTick->CTRL = 0x00;*/              //disable SysTick

               if((flag != previous_flag)&& flag < 20){
                   P4->OUT ^= BIT7;
                   TIMER32_2->LOAD = 750000;
                   previous_flag++;
               }
               else if(flag == 20){
                   flag = 0;
                   P4->OUT |= BIT7;                     //turn pedestrian warning ON
                   TIMER32_2->LOAD = 9000000;           //3 sec count for EYellow
                   North_pedestrian = 0;
                   state = EYellow;
               }
               break;
        }
    }
}
