/* amp-supervisor.c
 * Use the PIC16F1769-I/P as a supervisor on TFG amplifier board.
 * PJ 2023-02-24
 */
// Configuration Bit Settings (generated from Config Memory View)
// CONFIG1
#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF
#pragma config IESO = ON
#pragma config FCMEN = ON

// CONFIG2
#pragma config WRT = OFF
#pragma config PPS1WAY = OFF
#pragma config ZCD = OFF
#pragma config PLLEN = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOR = OFF
#pragma config LVP = ON

#include <xc.h>
#include "./global_defs.h"

#define LED LATCbits.LATC5
#define SW0 PORTAbits.RA2
#define SW1 PORTAbits.RA5
#define SW2 PORTAbits.RA4
#define CSAn LATBbits.LATB5
#define CSBn LATCbits.LATC0

void init_mcu(void)
{
    OSCCONbits.IRCF = 0b1101; // FOSC 4 Mhz
    OPTION_REGbits.nWPUEN = 0; // Enable pull-ups
    //
    TRISCbits.TRISC5 = 0; LED = 0; // LED output
    ANSELAbits.ANSA2 = 0; TRISAbits.TRISA2 = 1; WPUAbits.WPUA2 = 1; // SW0 input
    TRISAbits.TRISA5 = 1; WPUAbits.WPUA5 = 1; // SW1 input
    ANSELAbits.ANSA4 = 0; TRISAbits.TRISA4 = 1; WPUAbits.WPUA4 = 1; // SW2 input
    //
    TRISCbits.TRISC4 = 1; WPUCbits.WPUC4 = 1; // Unused
    TRISBbits.TRISB7 = 1; WPUBbits.WPUB7 = 1; // Unused
    TRISAbits.TRISA0 = 1; WPUAbits.WPUA0 = 1; // PGD
    TRISAbits.TRISA1 = 1; WPUAbits.WPUA1 = 1; // PGC
    //
    ANSELBbits.ANSB6 = 0; TRISBbits.TRISB6 = 0; // SCK
    ANSELBbits.ANSB4 = 0; TRISBbits.TRISB4 = 1; WPUBbits.WPUB4 = 1; // SDI
    ANSELCbits.ANSC1 = 0; TRISCbits.TRISC1 = 0; // SDO
    ANSELBbits.ANSB5 = 0; TRISBbits.TRISB5 = 0; // CSAn
    ANSELCbits.ANSC0 = 0; TRISCbits.TRISC0 = 0; // CSBn
    //
    ANSELCbits.ANSC2 = 1; TRISCbits.TRISC2 = 1; // OPA1out
    ANSELCbits.ANSC3 = 1; TRISCbits.TRISC3 = 1; // OPA2out
    //
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    SSPCLKPPS = 0b01110; // RB6
    RB6PPS = 0b10010; // SCK
    SSPDATPPS = 0b01100; // SDI is RB4
    RC1PPS = 0b10100; // SDO
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
}

int main()
{
    unsigned char gain;
    init_mcu();
    // Read switches to select amplifier gain.
    gain = 0;
    if (SW0) gain |= 0b001;
    if (SW1) gain |= 0b010;
    if (SW2) gain |= 0b100;
    // Wait long enough after MCU reset that we can see the count clearly.
    __delay_ms(3000);
    // Blink LED to indicate value of gain.
    LED = 0;
    for (unsigned char i=0; i <= gain; ++i) {
        LED = 1;
        __delay_ms(900);
        LED = 0;
        __delay_ms(300);
    }
    __delay_ms(2000);
    // After the amplifiers are set up, turn on the LED and leave it on.
    LED = 1;
    while (1) { /* forever */ }
    return 0; // We should never arrive here.
}
