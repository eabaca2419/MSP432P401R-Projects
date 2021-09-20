/*Name: MSP432&ESP8266 Server
  Author: Erick Baca
  Date: Novemeber 20, 2020
*/

#include "msp.h"
#include "string.h"
#include "stdint.h"
#include <stdlib.h>
#include "des.h"

/* VCC = 3.3V
 * GND = GND
 * TX = P3.2
 * RX = P3.3
 * RST = P6.1
 * CH_PD = 3.3V */

/*
 7. Rx          * *   8. Vcc(+3.3 V)
 5. GPIO -0     * *   6. Reset
 3. GPIO -2     * *   4. CH_PD
 1. Ground      * *   2. TX
*/

#define SIZE 900
char bc[SIZE], x;
char tempBuffer[50];
char passcode[8];
unsigned char message[8];
int tic; //global variable to count clock cycles
int i;

//Initialize all Global variables
void InitVar();

// DES Setup
void DES();

//Terminal Prints
void PrintConnect();
void CLRScreen();
void SendBc();

// Wifi Setup
void WifiSetup();
void WifiNetwork (char *msg);
void SendPutty(char *message);
void Set12Mhz();
void PuttySetup();
void WifiCreate();

//Variable Setup for Motor
int pulse[] = {3000,3300,3600,3900,4200,4500,4800,5100,5400,5700,6000,6300,6600,6900,7200,7500,7800,8100,8400,8700,9000};

uint8_t ServoIndex = 0;                     //Array ServoIndex
uint8_t LIMIT;                              //Set the limit of the array
#define UP 1                                //Set macro as 1 as reference of up
#define DOWN 0                              //Set macro as 0 as reference of down
uint8_t ServoMotor;                         //Variable to hold the direction of the motion of the sensor

//Servo Motor Functions
void TimerA0Setup();                        //
void SysTickSetup();                        //
void PortSetup();                           //Enable the pins for PWM and LEDs
void ServoMotorSetup();

void TimerA1Setup();                        // Timer to

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;
    InitVar();                              // Initialize all Global variables at 0 and clear arrays
    PortSetup();                           // setup for Port 2

    Set12Mhz();
    PuttySetup();
    WifiSetup();
    TimerA0Setup();                         //Initialize PWM clock at 20ms|50Hz and dutyCycle at the pulse[ServoIndex ]
    SysTickSetup();                         //Set SysTick at 120ms roughly, to slow the duration of motor speed
    ServoMotorSetup();
    CLRScreen();


    WifiCreate();
    DES();

    TimerA1Setup();                               //FUNCTION TO CHECK IF A CLIENT CONECTS TO THE NETWORK
    // Enable global interrupt
    __enable_irq();

    while(1){
    }
}

void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
    {
        bc[x] = EUSCI_A2->RXBUF;
        x++;
    }
}
void TA1_0_IRQHandler(void){
    tic++;
    if(tic >= 200){
        PrintConnect();
        tic = 0;
    }
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

void SysTick_Handler(void){
    if(ServoMotor == DOWN){                 //If the motion is set as DOWN enter the statement
    ServoIndex++;                           //increment ServoIndex
        if(ServoIndex > LIMIT - 1){         //Once the ServoIndex reach it limit, change direction flag
            SysTick->CTRL &= ~0x07;         //Stop the interrupt
        }
    }
    if(ServoMotor == UP){                   //If the motion is set as UP
        ServoIndex--;                       //Decrement the ServoIndex
        if(ServoIndex == 0){
            SysTick->CTRL &= ~0x07;         //Stop the sysTick interrupt
        }
    }

    TIMER_A0->CCR[4] = pulse[ServoIndex] - 1; //set the new PWM value to the motor
}

void SendPutty(char *message){
    int x;
    for(x = 0; x < strlen(message); x++){
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = message[x];
    }
    return;
}
void WifiNetwork (char *msg){
    int x;
        for(x = 0; x < strlen(msg); x++){
            while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A2->TXBUF = msg[x];
        }
        return;
}

void CLRScreen(){
    SendPutty("\033[2J");
    SendPutty("\033[H");
 return ;
}

void SendBc(){
     SendPutty(bc);
     memset(bc,0,sizeof(bc));
     x = 0;
}
void WifiCreate(){

    WifiNetwork("AT+CWMODE=3\r\n");                                      //SERT MODE TO STATION AND SERVER
    __delay_cycles(12000000);
    SendBc();

    WifiNetwork("AT+CWJAP=\"MyWifi2\",\"itachi2419\"\r\n");             //Join your WIFI with correct name and password
    __delay_cycles(96000000);
    SendBc();

    WifiNetwork("AT+CIPAP=\"192.168.5.1\"\r\n");                         // CREATE IP ADDRESS
    __delay_cycles(12000000);
    SendBc();

    WifiNetwork("AT+CIFSR\r\n");                                         //INFO ABOUT NETWORK
    __delay_cycles(96000000);
    SendBc();

    WifiNetwork("AT+CWSAP=\"ESP8266\",\"ERIVES2419\",5,3\r\n");       //CREATE ESP'S NETWORK AND PASSWORD
    __delay_cycles(12000000);
    SendBc();

    WifiNetwork("AT+CIPMUX=1\r\n");                                      //SET FOR MULTIPLE CONECTIONS
    __delay_cycles(12000000);
    SendBc();


    WifiNetwork("AT+CIPSERVER=1,8080\r\n");                              //SET PORT NUMBER FOR TCP CONECTION
    __delay_cycles(12000000);
    SendBc();
}

void PrintConnect(){

    WifiNetwork("AT+CWLIF\n\r");

    if(strstr(bc,"CONNECT"))
    {
        SendPutty("Client has connected");
        SendPutty("\r\n");
    }
        if(strstr(bc,"+IPD"))
        {
            strcpy(tempBuffer,bc);
            memcpy(&passcode, tempBuffer + 31, sizeof(passcode));
            SendPutty("\r\n");
            SendPutty("Message received: ");
            SendPutty("\r\n");
            des_decrypt((unsigned char *)passcode,message);
            SendPutty((char *)message);
            SendPutty("\r\n");

            if(strstr((char *)message, "open") && ServoMotor != UP){ //Compare the newly created string to the
                                                                //keyword in order to open the door/ServoMotor
                ServoIndex = LIMIT - 1;                         //Set the ServoMotor as close
                P2OUT = 0x02;                                   //Turn on Green LED
                SysTick->CTRL |= 0x07;                          //Start the sysTick
                TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;            //Start the PWM clock to run and start the ServoMotor


            }
            else if(strstr((char *)message, "close") && ServoMotor != DOWN){ //Compare the newly created string to the                                                                         //keyword in order to close the door/ServoMotor
                P2OUT = 0x02;                                   //Turn on Green LED
                ServoIndex = 0;                                 //Set the ServoMotor at open
                SysTick->CTRL |= 0x07;                          //Start the sysTick
                TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;            //Start the PWM clock to run

            }
            else if(strstr((char *)message, "lightON")){
                P2OUT = 0x02;                                   //Turn on Green LED
                P1->OUT = 0x01;
            }
            else if(strstr((char *)message, "lightOFF")){
                P2OUT = 0x02;                                   //Turn on Green LED
                P1->OUT &= ~0x01;
            }
            else{   //If no Keywords have been input it the clock will stop
                P1->OUT &= ~0x01;
                P2OUT = 0x01;                                   //Turn on Red LED
                SendPutty("Wrong");
                TIMER_A0->CTL &= ~TIMER_A_CTL_MC_1;             //Clock stopped
            }
            memset(message,0,sizeof(message));
            SendPutty("\r\n");

        }
        memset(bc,0,sizeof(bc));
        x = 0;

}
void InitVar(){
    x = 0;
    i = 0;
    tic = 0;
    memset(bc,0,sizeof(bc));
    memset(tempBuffer,0,sizeof(tempBuffer));
    memset(passcode,0,sizeof(passcode));
    memset(message,0,sizeof(message));
    ServoIndex = 0;
    return;
}

void DES(){
    // Create DES config by copying the default config
    DES_Config cfg = DES_default;
    // Enable IP and FP
    cfg.iperm = 1;
    cfg.fperm = 1;
    // Customize number of rounds
    cfg.rounds = 16;

    // Key for 64bit key
    const unsigned char key[] = {0x3b, 0x38, 0x98, 0x37, 0x15, 0x20, 0xf7, 0x5e};
    des_init(key, cfg);
}

void Set12Mhz(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses
}

void ServoMotorSetup(){
    LIMIT = sizeof(pulse)/sizeof(int);      //Store in the variable the total values in the array
    ServoIndex = LIMIT - 1;                 //Initialize the ServoIndex as closed
    ServoMotor = 2;                         //Initialize motion to go up
}

void SysTickSetup(){
    SysTick->CTRL = 0x00;                   //Initialize the clock halting
    SysTick->VAL = 1;                       //Set value at 1
    SysTick->LOAD = (int)(375000/2) - 1;    //Set the sysTick around 120ms
}

void PortSetup(){
        P2->SEL0 |= 0x80;                   //Select 2.7 as PWM
        P2->SEL1 &= ~0x80;                  //Bit clear 2.7
        P2->DIR |= 0x87;                    //Set 2.7 as output and enable the Green, Red and Blue LEDs
        P1->DIR |= 0x01;
        P2->OUT &=~ 0x07;
        P1->OUT &=~ 0x01;
}

void TimerA0Setup(){
        TIMER_A0->CTL |= TIMER_A_CTL_MC_0 +TIMER_A_CTL_TASSEL_2 + TIMER_A_CTL_ID__4; //set the clock frequency
        TIMER_A0->CCR[0] = 60000 - 1;                                       //20ms or 50hz
        TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;                         //Set PWM
        TIMER_A0->CCR[4] = pulse[ServoIndex] - 1;                           //Initialize the PWM pin with the ServoIndex of the array
}

void TimerA1Setup(){
    TIMER_A1->CTL |= TIMER_A_CTL_MC_1 +TIMER_A_CTL_TASSEL_2;
    TIMER_A1->CCR[0] = 60000 - 1; //12mhz/60000
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE;
    NVIC->ISER[0] = 1 << (TA1_0_IRQn & 31 );
}

void PuttySetup(){
        // Configure UART pins
        P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

        // Configure UART
        EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
        EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
                EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
        // Baud Rate calculation
        // 12000000/(16*9600) = 78.125
        // Fractional portion = 0.125
        // User's Guide Table 21-4: UCBRSx = 0x10
        // UCBRFx = int ( (78.125-78)*16) = 2
        EUSCI_A0->BRW = 78;                     // 12000000/16/9600
        EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
                EUSCI_A_MCTLW_OS16;

        EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
        EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
        EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
        // Enable eUSCIA0 interrupt in NVIC module
        NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
    }

void WifiSetup(){
        // Configure UART pins
        P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

        // Configure UART
        EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
        EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
                EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
        // Baud Rate calculation
        // 12000000/(16*9600) = 78.125
        // Fractional portion = 0.125
        // User's Guide Table 21-4: UCBRSx = 0x10
        // UCBRFx = int ( (78.125-78)*16) = 2
        EUSCI_A2->BRW = 6;                     // 12000000/16/9600
        EUSCI_A2->MCTLW = (8 << EUSCI_A_MCTLW_BRF_OFS) |
                EUSCI_A_MCTLW_OS16;

        EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
        EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
        EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
        // Enable eUSCIA0 interrupt in NVIC module
        NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}
