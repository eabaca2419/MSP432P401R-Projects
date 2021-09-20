/*Name: MSP432&ESP8266 Client
  Authors: Erick Baca
  Date: November 22, 2020
*/

#include "msp.h"
#include "string.h"
#include "stdint.h"
#include <stdio.h>
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

#define size 900
#define size2 200

//Variables for WIFI
int FLAG;                               //When a carriage return or new line is detected
                                        //meaning their was an ENTER press in the terminal
char bc[size];                          //Array used to communicate UARTA2
char x;

//Variables for putty
char buffer[8];                         //Array where the terminal input is saved
uint8_t index;
int i;
unsigned char output[8];                //DES encrypt buffer contend
unsigned char message[8];

//Wifi Setup
void WifiSetup();                       //UARTA2 Setup
void WifiConnect();                     //Function to connect with the server
void SendWifi (char *msg);              //UART communication function
void SendMessage();                     //Function to send messages through the WIFI connection



//Putty Setup
void Set12Mhz();                        //Set to run the whole system in 12Mhz
void PuttySetup();                      //UARTA0 Setup
void TerminalRead(char c);              //Function to receive input from terminal
void SendPutty(char *message);          //Print in terminal screen
void SendEcryption( char text[16]);
void CLRScreen();                       //Clear screen
void Print_bc();                        //Print everything the bc transmits

//DES Setup
void DES();

// Initialize all variables
void InitVar();

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;      // Stop watchdog timer

    InitVar();                          //Initialize all variables at 0 and clear all strings
    Set12Mhz();                         //Set to run the whole system in 12Mhz

    PuttySetup();                       //UARTA0 Setup
    WifiSetup();                        //UARTA2 Setup
    CLRScreen();                        //Clear screen
    DES();                              //DES setup function

    WifiConnect();                      //Function to connect with the server


    SendPutty("Enter Password: ");

    __enable_irq();                     //Enable interrupts

  while(1){
      if(FLAG){                         //Check for flag which is activated in the UARTA0 interrupt once it detects \r or \n
          SendMessage();                //Send the message to server
          FLAG = 0;                     //Clear flag
      }
  }
}


// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{

    if(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG){
        buffer[index] = EUSCI_A0->RXBUF;

        TerminalRead(buffer[index]);    //The terminal is receiving input and its saving on the buffer

        if(buffer[index] == '\n' || buffer[index++] == '\r'){   //Looks in the buffer for a carriage return or new line
                                                                //meaning their was an ENTER press in the terminal
                SendPutty("\r\n");                              //Commence a new line in the terminal
                SendPutty("Received: ");                        //Present to the user what has being stored in the buffer
                SendPutty(buffer);                              //Print the buffer in the terminal
                SendPutty("\r\n");
                des_encrypt((unsigned char *)buffer, output);   //Encrypt the content of the buffer and saved in output

                /* This is to print the message encrypted but the message give certain commands in the terminal which messes
                 * with other variables in the code. If you want to test and see the message use "password" all lower case and
                 * it should encounter no error.

                SendPutty((char *)output);                      //Print the Encrypted message

                */

                SendPutty("\r\n");
                FLAG = 1;                                       //activate the flag and send the encrypted message
                memset(&buffer,0,sizeof(buffer));               //Clear the buffer
                index = 0;
        }
    }
}

void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
    {
        bc[x] = EUSCI_A2->RXBUF;
        x++;
        if(x == size){
            P1DIR = 0x01;
            P1OUT = 0x01;
        }
    }
}

void WifiConnect(){
    SendWifi("AT+CWMODE=1\r\n");                              //Set ESP8266 into Client mode
    __delay_cycles(12000000);
    Print_bc();

    SendWifi( "AT+CWJAP=\"ESP8266\",\"ERIVES2419\"\r\n");     //Join the WIFI server
    __delay_cycles(96000000);
    Print_bc();


    SendWifi("AT+CIFSR\r\n");                                 //Print the information about the network
    __delay_cycles(96000000);
    Print_bc();

    SendWifi("AT+CIPSTART=\"TCP\",\"192.168.0.20\",8080\r\n");//Start TCP connection with an static IP address
    __delay_cycles(96000000);
    Print_bc();

}

void SendMessage(){
    SendWifi("AT+CIPSEND=8\r\n");                             //Command for the ESP8266 to send the bits
    __delay_cycles(96000000);
    Print_bc();

    SendWifi((char *)output);                                 //Send the message through the TCP connection
    __delay_cycles(12000000);
    Print_bc();

    __delay_cycles(96000000);                                 //Give enough time in order for the bits to be send
    Print_bc();

    SendPutty("Enter Password: ");

//    SendWifi("AT+CIPCLOSE\r\n");                                  //CLOSE TCP CONECTION
//    __delay_cycles(96000000);
//    Print_bc();

}

void InitVar(){
    x = 0;
    i = 0;
    FLAG = 0;
    memset(bc,0,sizeof(bc));                //Clear bc
    memset(output,0,sizeof(output));        //Clear output
    memset(buffer,0,sizeof(buffer));        //clear buffer
    memset(message,0,sizeof(message));      //clear message
    index = 0;
    return;
}

void DES(){
    // Create DES config by copying the default config
    DES_Config cfg = DES_default;
    // Disable IP and FP
    cfg.iperm = 1;
    cfg.fperm = 1;
    // Customize number of rounds
    cfg.rounds = 16;

    // Key for 64bit key
    const unsigned char key[] = {0x3b, 0x38, 0x98, 0x37, 0x15, 0x20, 0xf7, 0x5e};
    des_init(key, cfg);
}

void CLRScreen(){
    SendPutty("\033[2J");                   //Clear output in the terminal
    SendPutty("\033[H");                    //Clear Terminal Window
 return ;
}

void Print_bc(){
    SendPutty(bc);
    memset(bc,0,sizeof(bc));
    x = 0;
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

void SendWifi (char *msg){
    int x;
        for(x = 0; x < strlen(msg); x++){
            while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A2->TXBUF = msg[x];
        }
        return;
}

void SendPutty(char *message){
    int i;
    for(i = 0; i < strlen(message); i++){
     while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF = message[i];
    }
    return;
}

void SendEcryption( char text[16]){
    int i;
    for(i =0; i < 16; i++)
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF = text[i];
    return;
}

void TerminalRead(char c){
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
           EUSCI_A0->TXBUF = c;
}

void PuttySetup(){
    // Configure UART pins
      P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
      P1->SEL1 &= ~(BIT2+BIT3);
      // Configure UART
      EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
      EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
                        EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
      EUSCI_A0->BRW = 78;                     // 12000000/16/9600
      EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16;
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

