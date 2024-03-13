/* 
 * File:   main.c
 * Author: joe
 *
 * Created on February 26, 2024, 5:12 PM
 */

// MESSAGE_START is defined in spi.h

// PIC18F4331 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = IRC        // Oscillator Selection bits (Internal oscillator block, CLKO function on RA6 and port function on RA7)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON     // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config FLTAMX = RC1     // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RC7      // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RC5 and RC4, respectively. SDO output is multiplexed with RC7.)
#pragma config PWM4MX = RB5     // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3    // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (Enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 4000000      // Also defined in XL5.c

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#include "../PIC18_xl5.X/xl5.h"         // XL5 library
#include "../PIC18_pc_pwm.X/pc_pwm.h"   // Power control pwm library
#include "../PIC18_spi.X/spi.h"         // Serial peripheral interface library
#include "../PIC18_adc.X/adc.h"         // Analog to digital converter library

void __interrupt( __high_priority ) h_isr( void );
int set_throttle(int, int);
void message_received(); 

struct adc_data throttle;       // Struct in PIC18_adc library  
struct xl5_data xl5_1;           // Struct in PIC18_xl5 library
struct spi_transmission message;

int main( int argc, char** argv ) {
    message.beginning_flag = 0;
    message.end_flag = 0;
    
    // set system clock to 4MHz
    IRCF0 = 0;                 
    IRCF1 = 1;
    IRCF2 = 1;
    
    // interrupt setup
    IPEN = 1;           // Set Interrupt Priority Enable
    GIE = 1;            // Global Interrupt Enable
    PEIE = 1;           // Peripheral Interrupt Enable
    SSPIE = 1;          // Synchronous Serial Port Interrupt Enable
    
    init_XL5();         // Initialize XL5 electronic speed controller
    PWM_init();         // Initialize PWM module
    spi_slave_init();   // Initialize SPI module as slave device

    xl5_1.drive_dir = 1;    // needed bc throttle comes before drive direction, could change this
    // main loop
    while( 1 ) 
    {
        if ( throttle.full_result_flag )    // If both bytes of ADC data have been received
        {
            throttle.full_result_flag = 0;
            throttle.full_result = ( throttle.res_hi << 8 ) + throttle.res_lo;      // Calculate full adc result
            PDC0L = set_throttle( xl5_1.drive_dir, throttle.full_result );             // Set PDCOL (pwm duty cycle)
        }
        if ( xl5_1.drive_dir_flag )             // If drive direction byte has been received       
        {
            xl5_1.drive_dir_flag = 0;           // Reset for next transmission    
        }
        if ( message.end_flag )
        {
            message.end_flag = 0;
            message_received();
        }
    }   
    return (EXIT_SUCCESS);
}

// high priority interrupts
void __interrupt( __high_priority ) h_isr( void ) 
{
    if ( SSPIE && SSPIF )               // If SPI buffer is full (data has been received)
    {
        if ( !message.beginning_flag )      // Expecting start message
        {
            if ( SSPBUF == MESSAGE_START ) // If start message was received
            {
                message.beginning_flag = 1; // Start message was received
                SSPIF = 0;              // Reception (of a byte) complete
                return;
            }
            else                        // Was expecting start message and got something else
            {
                SSPBUF = 0b00000000;    // Clear buffer
                return;
            }
        }
        else if ( !throttle.lob_flag )        // If low byte hasn't been received yet
        {
            throttle.res_lo = SSPBUF;        // Then data sent was low byte
            throttle.lob_flag = 1;           // Low byte has been received  
            SSPIF = 0;                  // Reception (of a byte) complete
            return;
        }
        else if ( !throttle.hib_flag )       // If high byte hasn't been received yet
        {
            throttle.res_hi = SSPBUF;        // Then data sent was high byte
            throttle.hib_flag = 1;           // Reset flag for next receive
            throttle.full_result_flag = 1;      // ADC data has been received
            SSPIF = 0;                  // Reception (of a byte) complete
            return;
        }
        else if ( !xl5_1.drive_dir_flag )    // If drive direction hasn't been received yet
        {
            xl5_1.drive_dir = SSPBUF;         // Then data sent was drive direction
            message.end_flag = 1;  // Entire message has been received, reset byte flags    
            xl5_1.drive_dir_flag = 1;   // Drive direction has been received
            SSPIF = 0;                  // Reception (of a byte) complete
            return;
        }
    }
    return;    
}

int set_throttle( int forward_flag, int adc_result )
{
    int i;
    int step_size = 0;
    uint8_t dir_div = 0;                    // "resolution of the direction" 
                                            // uneven between forward and reverse because my pwm signal isn't exactly perfect
    uint8_t neutral = 95;                   // neutral throttle
    int curr_step = 0;
    
    if ( forward_flag )                     // forward
    {
        dir_div = 33;
        step_size = ( 1 << 10 ) / dir_div;  //  divide up largest AD result by the direction resolution
    }
    else                                    // reverse
    {
        dir_div = 31;
        step_size = ( 1 << 10 ) / dir_div;  //  divide up largest AD result by the direction resolution
    }
    
    for ( i = 1; i <= dir_div + 1; i++ )
    {
        curr_step = i * ( step_size );
        if ( curr_step >= adc_result )
        {
            if ( forward_flag == 1 )        // forward
            {
                return neutral + ( i - 1 ); // add to neutral throttle
            }
            else                            // reverse
            {
                return neutral - ( i - 1);  // subtract from neutral throttle
            }
        }
    }   
}

// Reset received flags for next transmission
void message_received()
{
    throttle.lob_flag = 0;
    throttle.hib_flag = 0;
    xl5_1.drive_dir_flag = 0;
    message.beginning_flag = 0;
}
