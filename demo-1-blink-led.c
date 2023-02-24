/* demo-1-blink-led.c
 * First code to bring up PIC16F1769-I/P on TFG amplifier board.
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

int main()
{
    OSCCONbits.IRCF = 0b1101; // FOSC 4 Mhz
    TRISCbits.TRISC5 = 0;
    LED = 0;
    while (1) {
        __delay_ms(500);
        LED ^= 1;
    }
    return 0; // We should never arrive here.
}
