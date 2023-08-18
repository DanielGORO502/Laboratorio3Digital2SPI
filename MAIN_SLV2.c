
/* UNIVERSIDAD DEL VALLE DE GUATEMALA
 * DEPARTAMENTO DE INGENIERIA ELCTRONICA & MECATRONICA
 * CURSO: ELECTRONICA DIGITAL 2
 * LABORATORIO No.3
 * 
 * File:   MAIN_SLV2_LAB3.c
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
#include "SPI.h"                   // AGREGAMOS LA LIBRERIRA DEL SPI.
#include "ADC_LIB.h"               // AGREGAMOS LA LIBRERIRA DE LA LCD.
#include "oscilador_config.h"      // LIBRERIA PARA CONFIGURACION DEL OSCILADOR.

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// DIRECTIVAS DEL COPILADOR
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#define _XTAL_FREQ 8000000         /* DEFINIMOS LA FRECUNCIA DE RELOJ, 
                                   PARA PODER USAR LOS DELAY.*/
//______________________________________________________________________________
// DECLARACION DE VARIABLES
//______________________________________________________________________________

uint8_t temporal = 0;
uint8_t V;

//______________________________________________________________________________
// PROTOTIPOS DE FUNCIONES
//______________________________________________________________________________

void SETUP(void);
void DECIMAL(uint8_t V, uint8_t SELEC); 

//______________________________________________________________________________
// FUNCION DE INTERRUPCIONES
//______________________________________________________________________________

void __interrupt() isr(void){

//----------------------------- INTERRUPCION SPI -------------------------------     

   if(SSPIF == 1){               // REVIDAMOS SI HEMOS RECIBIDO ALGUN DATO.      
        temporal = spiRead();    // ASIGNAMOS EL DATO A UNA VARIBLE TEMPORAL.
        spiWrite(V);             // ENVIAMOS DATOS AL MASTER. 
        SSPIF = 0;               // APAGAMOS LA BANDERA DEL SPI.
    }
   
//----------------------------- INTERRUPCION ADC -------------------------------     
    
  if(ADIF == 1){               // REVISAMOS LA BANDERA DEL ADC.   
      if(ADCON0bits.CHS == 0){ // REVISAMOS SI ESTA EN EL CANAL 0. 
         V = adc_read();}      // ASIGNAMOS EL VALOR CONVERTEIDO A UNA VARIABLE.              
      }  
}

//______________________________________________________________________________
// FUNCION PRINCIPAL (MAIN & LOOP)
//______________________________________________________________________________

void main(void) {

    SETUP();
//______________________________ LOOP INFINITO _________________________________     

    while(1){

//------------------------------- REVISION ADC ---------------------------------      
         
        if(ADCON0bits.GO == 0){        // REVISAMOS SI YA TERMINO LA CONVERSION.  
            __delay_us(50);
            ADCON0bits .GO = 1;        // ENCENDEMOS LA CONVERSION NUEVAMENTE.     
        }         
    }
    return;
}

//______________________________________________________________________________
// FUNCION DE CONFIGURACION
//______________________________________________________________________________

void SETUP(void){

//------------------- CONFIGURACION DE ENTRADAS Y SALIDAS ----------------------
 
    ANSEL  = 0x3;            // HABILITAMOS LAS ENTRADAS ANALOGICAS AN1 & AN2.   
    ANSELH = 0x3;
    
    TRISA = 0x21;            // DECLARAMOS EL PIN 0 Y 5 DEL PORTA COMO ENTRADAS.   
    
    PORTA = 0x00;            // LIMPIAMOS LOS PUERTOS. 
    
//---------------------- CONFIGURACION DE RELOJ A 8MHZ -------------------------

  int_osc_MHz(3);            // 8 MHz 

//-------------------------- CONFIGURACION DE ADC ------------------------------
    
    adc_init(2,0,0);         // (FOSC/32), VDD, VSS. 
    adc_start(0);            // CHANEL 0, ENCENDEMOS ADC, INICIAMOS CONVERSION. 

//-------------------------- CONFIGURACION DEL SPI -----------------------------
    
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);   
    
//---------------------- CONFIGURACION DE INTERRUPCIONES -----------------------
      
    INTCONbits.GIE  = 1;     // HABILITAMOS LA INTERRUPCION GLOBALES.
    INTCONbits.PEIE = 1;     // HABILITAMOS LA INTERRUPCION PEIE
    PIR1bits.SSPIF  = 0;     // BORRAMOS LA INTERRUPCION MSSP
    PIE1bits.RCIE   = 1;
    PIR1bits.ADIF   = 0;     // LIMPIAMOS LA BANDERA DEL ADC.  
}
//______________________________________________________________________________
//______________________________________________________________________________
