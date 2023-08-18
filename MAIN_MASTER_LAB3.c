
/* UNIVERSIDAD DEL VALLE DE GUATEMALA
 * DEPARTAMENTO DE INGENIERIA ELCTRONICA & MECATRONICA
 * CURSO: ELECTRONICA DIGITAL 2
 * LABORATORIO No.3
 * 
 * File:   MAIN_MASTER_LAB3.c
 * Author: DANIEL GONZALEZ
 *
 * Created on August 8, 2023, 9:09 AM
 */

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// CONFIG1
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#pragma config FOSC  = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE  = OFF      // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP    = OFF      // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD   = OFF      // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO  = OFF      // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP   = OFF      // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// CONFIG2
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT   = OFF      // Flash Program Memory Self Write Enable bits (Write protection off)

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// LIBRERIAS
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#include <xc.h>
#include <stdint.h>
#include "oscilador_config.h"   // LIBRERIA PARA CONFIGURACION DEL OSCILADOR.
#include "LCD.h"                // AGREGAMOS LA LIBRERIRA DE LA LCD.
#include "SPI.h"                // AGREGAMOS LA LIBRERIRA DEL SPI.

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// DIRECTIVAS DEL COPILADOR
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#define _XTAL_FREQ 8000000      /* DEFINIMOS LA FRECUNCIA DE RELOJ, 
                                   PARA PODER USAR LOS DELAY.*/

//______________________________________________________________________________
// DECLARACION DE VARIABLES
//______________________________________________________________________________

uint8_t SOL = 1;

uint16_t PT1;
uint16_t PT2;

uint8_t  UNID;
uint8_t  DECE;
uint8_t  CENT;

uint8_t  NUM_1;
uint8_t  NUM_2;
uint8_t  NUM_3;
uint8_t  NUM_4;
uint8_t  NUM_5;
uint8_t  NUM_6;
uint8_t  NUM_7;
uint8_t  NUM_8;
uint8_t  NUM_9;

uint8_t  CONT;


//______________________________________________________________________________
// PROTOTIPOS DE FUNCIONES
//______________________________________________________________________________

void SETUP(void);
void DECIMAL(uint8_t V, uint8_t SELEC); 

//______________________________________________________________________________
// FUNCION DE INTERRUPCIONES
//______________________________________________________________________________

//______________________________________________________________________________
// FUNCION PRINCIPAL (MAIN & LOOP)
//______________________________________________________________________________

void main(void) {
    
    SETUP();    
//______________________________ LOOP INFINITO _________________________________     
    
    while(1){
        
//--------------------------- SLAVE ONE (SENSOR 1) -----------------------------
        
       PORTCbits.RC2 = 0;       // SELECCIONAMOS EL SLAVE 1.
       __delay_ms(1);
       
       spiWrite(0);             // SOLICITAMOS DATOS AL SLAVE 1.
       PT1 = spiRead();         // LECTRURA DE LOS DATOS ENVIADOS POR SLAVE 1.
       DECIMAL(PT1,0);          // CONVERSION DEL DATO PARA MOSTRARLO EN LA LCD.
       __delay_ms(1);
       
       PORTCbits.RC2 = 1;       // DESELECCIONAMOS EL SLAVE 1. 
       __delay_ms(1);
       
//--------------------------- SLAVE TWO (SENSOR 2) -----------------------------
                          
       PORTCbits.RC1 = 0;       // SELECCIONAMOS EL SLAVE 2. 
       __delay_ms(1);
       
       spiWrite(SOL);           // SOLICITAMOS DATOS AL SLAVE 2.
       PT2 = spiRead();         // LECTRURA DE LOS DATOS ENVIADOS POR SLAVE 2.
       DECIMAL(PT2,1);          // CONVERSION DEL DATO PARA MOSTRARLO EN LA LCD. 
       __delay_ms(1);
       
       PORTCbits.RC1 = 1;       // DESELECCIONAMOS EL SLAVE 2.
       __delay_ms(1);     

//--------------------------- SLAVE ONE (CONTADOR) -----------------------------      
        
       PORTCbits.RC2 = 0;       // SELECCIONAMOS EL SLAVE 1.
       __delay_ms(1);
       
       spiWrite(1);             // SOLICITAMOS DATOS AL SLAVE 1.
       CONT = spiRead();        // LECTRURA DE LOS DATOS ENVIADOS POR SLAVE 1.
       PORTA = CONT;
       DECIMAL(CONT,2);         // CONVERSION DEL DATO PARA MOSTRARLO EN LA LCD.
       
       __delay_ms(1);
       
       PORTCbits.RC2 = 1;       // DESELECCIONAMOS EL SLAVE 1. 
       __delay_ms(1);
       
//------------------------ ESCRITURA EN PANTALLA LCD ---------------------------         

        Lcd_Set_Cursor(1,1);    
        Lcd_Write_String("S1");      // MOSTRAMOS EL NOMBRE DEL SENSOR "S1".   
        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char(NUM_1);       // NUMERO ENTERO POT1. 
        Lcd_Write_Char(46);          // "."
        Lcd_Write_Char(NUM_2);       // PRIMER DECIMAL POT1. 
        Lcd_Write_Char(NUM_3);       // SEGUNDO DECIMAL POT1.
        
        
        Lcd_Set_Cursor(1,7);
        Lcd_Write_String("S2");      // MOSTRAMOS EL NOMBRE DEL SENSOR "S2".
        Lcd_Set_Cursor(2,7);          
        Lcd_Write_Char(NUM_4);       // NUMERO ENTERO POT2. 
        Lcd_Write_Char(46);          // "."
        Lcd_Write_Char(NUM_5);       // PRIMER DECIMAL POT2.  
        Lcd_Write_Char(NUM_6);       // SEGUNDO DECIMAL POT2.
    
       // DECIMAL(CONT,2);
        Lcd_Set_Cursor(1,13);
        Lcd_Write_String("S3:");      // Colocamos el nombre del sensor "S3".
        Lcd_Set_Cursor(2,13);          
        Lcd_Write_Char(NUM_7);           // Mostramos el valor del contador.    
        Lcd_Write_Char(NUM_8);           // Mostramos el valor del contador.
        Lcd_Write_Char(NUM_9);           // Mostramos el valor del contador.
        
    }
    return;
}
//______________________________________________________________________________
// FUNCION DE SEPARACION DE DIGITOS Y CONVERSION ASCII.
//______________________________________________________________________________

void DECIMAL(uint8_t V, uint8_t SELEC ){

//-------------------- DECLARACION DE VARIABLES LOCALES ------------------------
    
    uint16_t VOLT;

//-------------------------- SEPARCION DE DIGITOS ------------------------------
   
    VOLT = (uint16_t)(V*1.961);          // MAPEAMOS EL VALOR ENTRE 0V a 5V.
    if(SELEC ==2){
        VOLT = V;
    } 
    CENT = VOLT/100;                         // SEPARAMOS EL PRIMER DIGITO.
    DECE = (VOLT - CENT*100)/10;             // SEPARAMOS EL SEGUNDO DIGITO.
    UNID = (VOLT - CENT*100 - DECE*10)/1;    // SEPARAMOS EL TERCER DIGITO.

//--------------- CONVERSION ASCII & SELECCION DE POTENCIOMETRO ----------------
     
    if(SELEC == 0){
        NUM_1 = CENT + 0x30;
        NUM_2 = DECE + 0x30;
        NUM_3 = UNID + 0x30;  
    }
    
    if(SELEC == 1){
        NUM_4 = CENT + 0x30;
        NUM_5 = DECE + 0x30;
        NUM_6 = UNID + 0x30;  
    }
    
    if(SELEC == 2){
        NUM_7 = CENT + 0x30;
        NUM_8 = DECE + 0x30;
        NUM_9 = UNID + 0x30;  
    }
}        

//______________________________________________________________________________
// FUNCION DE CONFIGURACION
//______________________________________________________________________________

void SETUP(void){

//------------------- CONFIGURACION DE ENTRADAS Y SALIDAS ----------------------
    
    ANSEL  = 0x00;              // SIN ENTRADAS ANALOGICAS, SOLO DIGITALES.   
    ANSELH = 0x00;      
    
    TRISC1 = 0x00;              // DECLARAMOS EL PIN 1 DEL PORTC COMO SALIDA.
    TRISC2 = 0x00;              // DECLARAMOS EL PIN 2 DEL PORTC COMO SALIDA.   
    
    TRISA = 0x00;
    TRISB = 0x00;               // DECLARAMOS EL PORTB COMO SALIDAS.
    TRISD = 0x00;               // DECLARAMOS EL PORTD COMO SALIDAS.

    PORTA = 0x00;
    PORTB = 0x00;               // LIMPIAMOS LOS PUERTOS.
    PORTD = 0x00;  
        
//---------------------- CONFIGURACION DE RELOJ A 8MHZ -------------------------
    
    int_osc_MHz(3);             // 8 MHz     

//-------------------------- CONFIGURACION DE LCD ------------------------------

    unsigned int a;
    Lcd_Init();    

//-------------------------- CONFIGURACION DEL SPI -----------------------------

    PORTCbits.RC1 = 1;          // APAGAMOS LA COMUNICACION CON EL SLAVE 1.
    PORTCbits.RC2 = 1;          // APAGAMOS LA COMUNICACION CON EL SLAVE 2.
    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
}
//______________________________________________________________________________
//______________________________________________________________________________
