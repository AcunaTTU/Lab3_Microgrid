
// PIC32MX250F128B Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
//#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
//#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
//#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
//#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
//#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
//#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
//#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
//#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
//#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
//#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
//#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
//#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
//#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
//#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
//#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
//#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
//#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include<sys/attribs.h>
        
#define SYSCLK (40000000)
#define AN1Multiplier 0.02555
#define AN2Multiplier 0.02777
#define AN3Multiplier 0.02785
#define AN4Multiplier 0.0255
#define AN5Multiplier 0.02555
//#define RB4Multiplier 0.02775
//#define RB5Multiplier 0.02775

/* SerialTransmit() transmits a string to the UART2 TX pin MSB first
 *
 * Inputs: *buffer = string to transmit */
bool BattStatus = 0;
bool PanelStatus = 0;
bool Load1Status = 0;
bool Load2Status = 0;
bool VariableLdStatus = 0;

#define setPR2(seconds)   (seconds * SYSCLK / 256)
#define PWM_FREQ    100000
//#define DUTY_CYCLE  50
int DUTY_CYCLE = 0;
double voltage = 0;
int adc_value;




void ToggleRelay(int Relay){
    if (Relay == 1){
        printf("Toggle 1");
        LATBbits.LATB5 = !LATBbits.LATB5;
        if(LATBbits.LATB5 == 1){
            BattStatus = 1;
        }
        else BattStatus = 0;
        
    } 
    if (Relay == 2){
        LATBbits.LATB8 = !LATBbits.LATB8;
    } 
    if (Relay == 3){
        LATBbits.LATB9 = !LATBbits.LATB9;
    } 
    if (Relay == 4){
        LATBbits.LATB13 = !LATBbits.LATB13;
    } 
    if (Relay == 5){
        LATBbits.LATB14 = !LATBbits.LATB14;
    } 
}
int SerialTransmit(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while( size)
    {
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // send single character to transmit buffer
 
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U2STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
/* SerialReceive() is a blocking function that waits for data on
 *  the UART2 RX buffer and then stores all incoming data into *buffer
 *
 * Note that when a carriage return '\r' is received, a nul character
 *  is appended signifying the strings end
 *
 * Inputs:  *buffer = Character array/pointer to store received data into
 *          max_size = number of bytes allocated to this pointer
 * Outputs: Number of characters received */
unsigned int SerialReceive(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 
    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while(num_char < max_size)
    {
        while( !U2STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U2RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\r'){
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
} // END SerialReceive()
 
int analogRead(char analogPIN){
    AD1CHS = analogPIN << 16;       // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;           // Begin sampling
    while( AD1CON1bits.SAMP );      // wait until acquisition is done
    while( ! AD1CON1bits.DONE );    // wait until conversion done
 
    return ADC1BUF0;                // result stored in ADC1BUF0
}
 
void adcConfigureManual(){
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
 
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD
} // END adcConfigureManual()

void _mon_putc (char c)
 {
   while (U2STAbits.UTXBF); // Wait til current transmission is complete
   U2TXREG = c;
} 

void initUart(){
    RPB10R = 0b0010; //Set pin 21 to Transmit
    U2RXR = 0b0011;    //SET RX to RB11
    U2BRG = 259; //9600 BAUD from datasheet
    U2MODE = 0; //Enable UART 8N1
    U2STA = 0x1400; //Enable U2TX and RX
    U2MODE = 0x8000; // Enable UART2
}

// Timer2 Interrupt Service Routine
void __ISR(_TIMER_2_VECTOR, IPL1SOFT) Timer2IntHandler(void){
    // note that iplx (interrupt priority level) must match the timers interrupt priority level
    
    adc_value = analogRead(9); // Read pin AN5 (RB3 - Pin 7)
    DUTY_CYCLE = (adc_value/10.23);
    
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);
    
    // Reset the Timer 2 interrupt flag
    IFS0bits.T2IF = 0;
} // END Timer2 ISR

int main( void)

{
    //Timer 2 interrupt setup***************************************************
    T2CONbits.ON = 0;           //Timer Off
    T2CONbits.TCS = 0;          //0 = Internal peripheral clock source
    T2CONbits.T32 = 0;          // Odd numbered and even numbered timers form a separate 16-bit timer
    T2CONbits.TCKPS = 0b001;    // 1:2 prescale
    T2CONbits.TGATE = 0;        //0 = Gated time accumulation is disabled
   
     
    /* Initialize Timer 2 Interrupt Controller Settings */
    // Set the interrupt priority to 4
    IPC2bits.T2IP = 1;
    // Reset the Timer 2 interrupt flag
    IFS0bits.T2IF = 0;
    // Set Interrupt Controller for multi-vector mode 
    INTCONbits.MVEC = 1;
    // Enable interrupts from Timer 2
    IEC0bits.T2IE = 1;
    // Enable the peripheral 
    T2CONbits.ON = 1;
    // Enable Interrupt Exceptions 
    // set the CP0 status IE bit high to turn on interrupts globally
    __builtin_enable_interrupts();
//End Timer Interrupt Setup*************************************************
    
    
//PWM Setup*****************************************************************
    // Set OC1 to pin RB7 with peripheral pin select
    RPB7Rbits.RPB7R = 0x0005;
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] - 1
    PR2 = (SYSCLK / (PWM_FREQ * 2)) - 1;
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
//End PWM Setup*************************************************************

    // Configure pins as analog inputs
    ANSELBbits.ANSB15 = 1; // set RB15 (AN9) to analog
    TRISBbits.TRISB15 = 1; // set RB15 as an input (Pin 7)
    //TRISBbits.TRISB5 = 0;   // set RB5 (Pin 14) as an output 
    //LATBbits.LATB5 = 1;     //set RB5 high
    //adcConfigureManual(); // Configure ADC
    //AD1CON1SET = 0x8000; // Enable ADC
    initUart();
    // Configure pins as analog inputs
    //ANSELAbits.ANSA0 = 1;   // set RA0 (AN0) to analog
    ANSELAbits.ANSA1 = 1;   // set RA1 (AN1) to analog
    ANSELBbits.ANSB0 = 1;   // set RB0 (AN2) to analog
    ANSELBbits.ANSB1 = 1;   // set RB1 (AN3) to analog
    ANSELBbits.ANSB2 = 1;   // set RB2 (AN4) to analog
    ANSELBbits.ANSB3 = 1;   // set RB3 (AN5) to analog
    //ANSELBbits.ANSB15 = 1;   // set RB15 (AN9) to analog
    //ANSELBbits.ANSB14 = 1;   // set RB14 (AN10) to analog
    //ANSELBbits.ANSB13 = 1;   // set RB13 (AN11) to analog
    
    //TRISAbits.TRISA0 = 1;   // set RA0 as an input (Pin 2)
    TRISAbits.TRISA1 = 1;   // set RA1 as an input (Pin 3)
    TRISBbits.TRISB0 = 1;   // set RB0 as an input (Pin 4)
    TRISBbits.TRISB1 = 1;   // set RB1 as an input (Pin 5)
    TRISBbits.TRISB2 = 1;   // set RB2 as an input (Pin 6)
    TRISBbits.TRISB3 = 1;   // set RB3 as an input (Pin 7)
    //TRISBbits.TRISB15 = 1;   // set RB3 as an input (Pin 26)
    TRISBbits.TRISB5 = 0;   // set RB5 (Pin 14) as an output 
    TRISBbits.TRISB8 = 0;   // set RB5 (Pin 14) as an output 
    TRISBbits.TRISB9 = 0;   // set RB5 (Pin 14) as an output 
    TRISBbits.TRISB14 = 0;   // set RB3 as an input (Pin 25)
    TRISBbits.TRISB13 = 0;   // set RB3 as an input (Pin 24)
    
    LATBbits.LATB5 = 1;     //set RB5 high
    LATBbits.LATB8 = 1;     //set RB5 high
    LATBbits.LATB9 = 1;     //set RB5 high
    LATBbits.LATB13 = 1;     //set RB5 high
    LATBbits.LATB14 = 1;     //set RB5 high
    BattStatus = 1;
    PanelStatus = 1;
    Load1Status = 1;
    Load2Status = 1;
    VariableLdStatus = 1;
    
    
    
    adcConfigureManual();   // Configure ADC
    AD1CON1SET = 0x8000;    // Enable ADC
 
    //int adc_value;
    double Battery;
    double Panel;
    double Load1;
    double Load2;
    double VariableLd;
    int i;
    char   buf[1024];       // declare receive buffer with max size 1024
    unsigned int rx_size;
    
    //SerialTransmit("Hello Earthling!\r\n");
    //SerialTransmit("Talk to me and hit 'enter'. Let the mocking begin!\r\n\r\n");
 
    
	while (1)
	{
        //adc_value = analogRead(9); // Read pin AN5 (RB3 - Pin 7)
        for(i = 0; i < 350000; i++){}
        //adc_value = analogRead(5); // Read pin AN5 (RB3 - Pin 7)
        
        printf("\n");
        printf("\n");
        //adc_value = analogRead(1);
       
        
        Battery = analogRead(2) * AN2Multiplier;// 0.019686; //0.00317;
        //for(i = 0; i < 100000; i++){}
        //printf("Battery Voltage: ");
        printf("Battery Voltage:   %.2f\n", Battery);
        //printf("%.2f\n", Battery);
        //for(i = 0; i < 50000; i++){}
        
        
        //adc_value = analogRead(2);
		Panel = analogRead(3) * AN3Multiplier;// 0.019686; //0.00317;
		printf("Panel Voltage:     %.2f\n", Panel);
        //for(i = 0; i < 50000; i++){}
        
        //adc_value = analogRead(3);
        Load1 = analogRead(1) * AN1Multiplier;// 0.019686; //0.00317;
		printf("Load 1 Voltage:    %.2f\n", Load1);
        //for(i = 0; i < 50000; i++){}
        
        //adc_value = analogRead(4);
        Load2 = analogRead(4) * AN4Multiplier;// 0.019686; //0.00317;
		printf("Load 2 Voltage:    %.2f\n", Load2);
        //for(i = 0; i < 50000; i++){}
        
        /*
        //adc_value = analogRead(5);
        VariableLd = analogRead(5) * AN5Multiplier;// 0.019686; //0.00317;
		printf("Variable Load  Voltage: %.2f\n", VariableLd);
        //for(i = 0; i < 50000; i++){}
         */
        
        if (Battery > 0.05){
          printf("\n1.Battery:       ON\n");
        }
        else {
            printf("\n1.Battery:      OFF\n");
        }
        
        if (Panel > 0.05){
          printf("\n1.Solar Panel:   ON\n");
        }
        else{
            printf("\n1.SolarPanel:   OFF\n");
        }
        
        if (Load1 > 0.05){
          printf("\n1.Load 1:       ON\n");
        }
        else{
            printf("\n1.Load 1:       OFF\n");
        }
        
        if (Load2  > 0.05){
          printf("\n1.Load 2:       ON\n");
        }
        else{
            printf("\n1.Load 2:       OFF\n");
        }
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        
        
 
    }//END while( 1)
    
    return 0;
}


 

 
