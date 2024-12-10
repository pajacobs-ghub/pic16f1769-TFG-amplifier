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
#define ACHANA 8
#define ACHANB 9

void init_mcu(void)
{
    OSCCONbits.IRCF = 0b1101; // FOSC 4 Mhz
    OPTION_REGbits.nWPUEN = 0; // Enable pull-ups
    //
    // MCU pins
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
    ANSELBbits.ANSB6 = 0; TRISBbits.TRISB6 = 0; LATBbits.LATB6 = 0; // SCK
    ANSELBbits.ANSB4 = 0; TRISBbits.TRISB4 = 1; WPUBbits.WPUB4 = 1; // SDI
    ANSELCbits.ANSC1 = 0; TRISCbits.TRISC1 = 0; LATCbits.LATC1 = 0; // SDO
    ANSELBbits.ANSB5 = 0; TRISBbits.TRISB5 = 0; CSAn = 1; // CSAn
    ANSELCbits.ANSC0 = 0; TRISCbits.TRISC0 = 0; CSBn = 1; // CSBn
    //
    ANSELCbits.ANSC2 = 1; TRISCbits.TRISC2 = 1; // OPA1out
    ANSELCbits.ANSC3 = 1; TRISCbits.TRISC3 = 1; // OPA2out
    //
    // Configure SPI
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
    //
    // From the MCP6S21 data sheet, choose to use SPI mode 0,0
    SSP1STATbits.SMP = 0; // Sample in middle of data output time
    SSP1STATbits.CKE = 1; // Transmit data on active to idle level of clock
    SSP1CON1bits.CKP = 0; // Clock idles low
    SSP1CON1bits.SSPM = 0b0001; // Mode is master, clock is FOSC/16
    SSP1CON1bits.SSPEN = 1; // Enable
    //
    // Fixed voltage reference at 2.048V.
    FVRCONbits.ADFVR = 0b10;
    FVRCONbits.CDAFVR = 0b10;
    FVRCONbits.FVREN = 1;
    while (!FVRCONbits.FVRRDY) { /* wait */ }
    //
    // ADC
    // Channel A AN8 RC6
    // Channel B AN9 RC7
    TRISCbits.TRISC6 = 1; ANSELCbits.ANSC6 = 1; WPUCbits.WPUC6 = 0;
    TRISCbits.TRISC7 = 1; ANSELCbits.ANSC7 = 1; WPUCbits.WPUC7 = 0;
    ADCON0bits.CHS = ACHANA;
    ADCON1bits.ADCS = 0b011; // FRC (also 0b111)
    ADCON1bits.ADFM = 1; // Right justified
    ADCON1bits.ADNREF = 0; // Vss
    ADCON1bits.ADPREF = 0b11; // FVR
    PIR1bits.ADIF = 0;
    ADCON0bits.ADON = 1;
    //
    // DACs to provide the reference voltages for the PGAs.
    DAC1CON0bits.DAC1FM = 0; // right justified
    DAC1CON0bits.PSS = 0b10; // FVR_buffer2
    DAC1CON0bits.NSS = 0; // Vss
    DAC1CON0bits.DACOE = 0; // Don't output to the external pin
    DAC1CON0bits.EN = 1;
    //
    DAC2CON0bits.DACFM = 0; // right justified
    DAC2CON0bits.PSS = 0b10; // FVR_buffer2
    DAC2CON0bits.NSS = 0; // Vss
    DAC2CON0bits.DACOE = 0; // Don't output to the external pin
    DAC2CON0bits.EN = 1;
    //
    // MCU on-chip OpAmps to buffer the DACs
    OPA1CONbits.ORM = 0b00; // Disable override function
    OPA1CONbits.UG = 1; // Unity gain
    OPA1PCHSbits.PCH = 0b0010; // DAC1_out
    OPA1CONbits.EN = 1; // Enable
    //
    OPA2CONbits.ORM = 0b00; // Disable override function
    OPA2CONbits.UG = 1; // Unity gain
    OPA2PCHSbits.PCH = 0b0011; // DAC2_out
    OPA2CONbits.EN = 1; // Enable
}

unsigned int read_adc(unsigned char chan)
{
    ADCON0bits.CHS = chan & 0b00011111;
    __delay_ms(1);
    PIR1bits.ADIF = 0;
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO_nDONE) { /* wait for conversion */ }
    PIR1bits.ADIF = 0;
    return ADRES;
}

void spi_send_gain(unsigned char g)
{
    unsigned char dummy;
    if (SSP1CON1bits.WCOL) SSP1CON1bits.WCOL = 0;
    // Send instruction byte followed by gain byte.
    PIR1bits.SSP1IF = 0;
    SSP1BUF = 0b01000000; // Instruction: write to gain register
    while (!PIR1bits.SSP1IF) { /* wait for transmission to complete */ }
    dummy = SSP1BUF; // Discard incoming data.
    PIR1bits.SSP1IF = 0;
    SSP1BUF = g; // Data: the gain byte
    while (!PIR1bits.SSP1IF) { /* wait for transmission to complete */ }
    dummy = SSP1BUF; // Discard incoming data.
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
    //
    // At this point, we assume that TFG circuits have settled.
    // Set the reference voltages for the amplifiers.
    // Note that PGA_A reference is provided by OPA2_out while
    // PGA_B reference is provided by OPA1_out.
    DAC2REF = read_adc(ACHANA); DACLDbits.DAC2LD = 1;
    DAC1REF = read_adc(ACHANB); DACLDbits.DAC1LD = 1;
    __delay_ms(1);
    //
    // Send gain to both PGAs.
    CSAn = 0; spi_send_gain(gain); CSAn = 1;
    CSBn = 0; spi_send_gain(gain); CSBn = 1;
    //
    // After the amplifiers are set up, turn on the LED and leave it on.
    __delay_ms(1);
    LED = 1;
    while (1) { /* forever */ }
    return 0; // We should never arrive here.
}
