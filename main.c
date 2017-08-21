/*

 * Original firmware written by
 *   Firmware Revision A - Written by Ahmad@UltraKeet.com.au
 *
 *  Modified by overflo and mind from metalab.at in June 2017.
 *
 *  This revision contains code by ente to add serial support (mostly for debugging)
 *
 * This firmware is heavily based on ahmads work and introduced the following changes:
 *  - some minor fixes for the cantina band and imperial march sounds
 *  - blinking the LED of the 2016.11 PCB revision
 *  - turning OFF the arc when connected via USB for charging.
 *    this is accomplished by using the internal votage reference set to 2.048 volt 
 *  - removed the eeprom stuff and made sure the cantina tune plays after 5 seconds of static plasma
 *
 * Original project can be found here:
 * http://ultrakeet.com.au/write-ups/modulated-arc-lighter
*/

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>


// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)





// Define our note indexes, the octaves are bullshit.

#define     SILENCE         0x00

#define     C1              0x01
#define     Db1             0x02
#define     D1              0x03
#define     Eb1             0x04
#define     E1              0x05
#define     F1              0x06
#define     Gb1             0x07
#define     G1              0x08
#define     Ab1             0x09
#define     A1              0x0A
#define     Bb1             0x0B
#define     B1              0x0C

#define     C2              0x0D
#define     Db2             0x0E
#define     D2              0x0F
#define     Eb2             0x10
#define     E2              0x11
#define     F2              0x12
#define     Gb2             0x13
#define     G2              0x14
#define     Ab2             0x15
#define     A2              0x16
#define     Bb2             0x17
#define     B2              0x18

#define     C3              0x19
#define     Db3             0x1A
#define     D3              0x1B
#define     Eb3             0x1C
#define     E3              0x1D
#define     F3              0x1E
#define     Gb3             0x1F
#define     G3              0x20
#define     Ab3             0x21
#define     A3              0x22
#define     Bb3             0x23
#define     B3              0x24



#define     NOTE_FULL       0xC0
#define     NOTE_HALF       0x80
#define     NOTE_QUARTER    0x40
#define     NOTE_EIGHTH     0x00

// Other definitions

#define     CLOCK_DIVIDER   15

// Array storing timer periods for the defined notes above

const unsigned char notes[36]={
    0xED, 0xE0, 0xD3, 0xC7, 0xBD, 0xB2, 0xA8, 0x9E, 0x96, 0x8D, 0x85, 0x7D,
    0x76, 0x70, 0x6A, 0x63, 0x5E, 0x59, 0x54, 0x4F, 0x4B, 0x46, 0x42, 0x3F,
    0x3B, 0x38, 0x34, 0x32, 0x2F, 0x2C, 0x2A, 0x27, 0x25, 0x23, 0x21, 0x1F
};

// Define our variables, again a lot of these are redundant
// and/or unused, meh.

unsigned char clockDivider=0;
unsigned pinState=0;

unsigned forceArc=0;
unsigned gate=0;
unsigned noGate=0;

unsigned postscaler=0;
unsigned int playIndex=0;
unsigned int genericDelay=0;



// Prototype our functions

void blockingDelay(unsigned int mSecs);
void playNote(unsigned char note, unsigned int duration);
void cantinaBand(void);
void imperialMarch(void);
void tetris(void);


// uart functions
char UART_WriteOnly_Init(const long int baudrate);
void UART_Write(char data);
void UART_Write_Text(char *text);
char UART_TX_Empty();

// voltage functions
void initVoltage();
unsigned int readVoltage();


void blinkLed()
{



LATAbits.LATA5 = 1; // for LED on
    blockingDelay(500);

LATAbits.LATA5 = 0; // for LED on
    blockingDelay(500);



}

// uart stuff
#define _XTAL_FREQ 8000000 // IRCF is set to 14 (1110b) which corresponds to 8MHz according to the manual -ente
char UART_WriteOnly_Init(const long int baudrate)
{
  unsigned int x;
  x = (_XTAL_FREQ - baudrate*64)/(baudrate*64);   //SPBRG for Low Baud Rate
  if(x>255)                                       //If High Baud Rage Required
  {
    x = (_XTAL_FREQ - baudrate*16)/(baudrate*16); //SPBRG for High Baud Rate
    BRGH = 1;                                     //Setting High Baud Rate
  }
  if(x<256)
  {
    SPBRG = x;                                    //Writing SPBRG Register
    SYNC = 0;                                     //Setting Asynchronous Mode, ie UART
    SPEN = 1;                                     //Enables Serial Port
    //TRISC7 = 1;                                   //As Prescribed in Datasheet
    //TRISC6 = 1;                                   //As Prescribed in Datasheet
    //CREN = 1;                                     //Enables Continuous Reception
    TXEN = 1;                                     //Enables Transmission
    return 1;                                     //Returns 1 to indicate Successful Completion
  }
  return 0;                                       //Returns 0 to indicate UART initialization failed
}
void UART_Write(char data)
{
  while(!TRMT);
  TXREG = data;
} 
void UART_Write_Text(char *text)
{
  int i;
  for(i=0;text[i]!='\0';i++)
    UART_Write(text[i]);
}
char UART_TX_Empty()
{
  return TRMT;
}
/// end of uart stuff



//called once after inite
void setup()
{


    // All pins to digital

    ANSELA=0x0000;

    // Set up our oscillator
    OSCCONbits.SCS=0x00;
    OSCCONbits.IRCF=14; // 8MHz
    OSCCONbits.SPLLEN=0x01;

    // Set up our timer
    TMR2IE = 1;
    
    
    PR2=0xED;
   
    //PR2=0x01;

    T2CONbits.T2CKPS=0x03;
    T2CONbits.TMR2ON = 1;

    // Enable timer interrupts
    TMR0IE=1;

    
  
    
    
    
    
    
    
    
    // Set up the option register
    OPTION_REG = 0x80 + 0x08;



    // Set up interrupts
    PEIE = 1;
    RCIE = 0;
    INTE = 0;
    GIE = 1;



/*
    TRISA1=0;
    TRISA2=0;
    TRISA3=0;
    TRISA4=0;

*/
    // ALL OUTPUTS (needed for led)
    TRISA = 0b11011111;  // RA5 output
    
    
    
    
  

    // Enable weak pull-up from the pushbutton
    WPUAbits.WPUA0=1;

    // Make all our pins digital IOs
    LATA0=0;
    LATA1=0;
    LATA2=0;
    LATA4=0;
    LATA5=0;





   // ANSEL = 0x00;   //disable all analog ports
   // ANSELH = 0x00;

    TRISAbits.TRISA2 = 1;   //Disable the output driver for pin RA2/AN2
    //ANSELbits.ANS2 = 1;     //set RA2/AN2 to analog mode
    //TRISBbits.TRISB7 = 0;   //set RB7 as an output


    ///////////////
    // ADC Setup //
    ///////////////
    //ADCON0bits.ADFM = 1;        //ADC result is right justified
    //ADCON0bits.VCFG = 0;        //Vdd is the +ve reference

    ADCON1bits.ADCS = 0b001;    //Fosc/8 is the conversion clock
                                //This is selected because the conversion
                                //clock period (Tad) must be greater than 1.5us.
                                //With a Fosc of 4MHz, Fosc/8 results in a Tad
                                //of 2us.
    ADCON0bits.CHS = 2;         //select analog input, AN2
    ADCON0bits.ADON = 1;        //Turn on the ADC



    ////////// ANALOGES ENDE



    UART_WriteOnly_Init(9600); // somehow 9600 is wrong (maybe because the XTAL value is wrong). Use 38400 8N1 -ente
    
    initVoltage();
}

#define DEBUG(s, t) {char buf[32]; sprintf(buf, s, t); UART_Write_Text(buf);}


unsigned int readVoltage()
{
    unsigned val;
    ADCON0bits.ADGO = 1; // Turn on ADC module           
    while(ADCON0bits.ADGO);
    val = (unsigned) ((ADRESH << 8 ) + ADRESL);
//    DEBUG("Got voltage: %u!\r\n", val);
    return val;
}

// configure ADC and FVR to compare VDD and a fixed control voltage of 2.048V
void initVoltage()
{   
    ADCON0bits.ADON = 1; // Turn on ADC module

    // Enable and configure FVR to 2.048 V for ADC
    FVRCON = 0b11000010 ;


   // enable FVR and set to 1.024 volt reference for ADC 
   //FVRCON = 0b11000001 ;    
    
    
    while(!FVRRDY); // wait for FVR to stabilize
    
    ADCON0bits.CHS = 0b11111; // select FVR as input for ADC
    ADCON1bits.ADPREF = 0b00; // use VDD as input for ADC
}


int checkUSBorLIPO()
{
 
  //  DEBUG("Hi, long long is %d bytes!\r\n",  sizeof(unsigned long long));

    unsigned long long voltages = 0;
    
  
  /*
    while(1)
    {
       
       voltages = readVoltage();
       DEBUG("V: %u\r\n", voltages); 
       blockingDelay(500);
       
    }
  
    */
    
    for (int i = 0; i < 5; i++) { readVoltage(); blockingDelay(10);} // read and toss 5 values
    for (int i = 0; i < 5; i++) voltages += readVoltage(); // compute average over the 5 next values
    DEBUG("\r\n\r\n(voltages/5): %u\r\n", (voltages/5));

    
    
    //return 1 if usb / >4.3Vv
    if((voltages/5) < 32232)
    {
        DEBUG("v < 32323 - 5V / USB .. BLINKING.\r\n", 0);
        return 1;
    }
    
    // if voltage lower 3.4v lipo needs charging, blink led.
    if((voltages/5) > 39232) 
    {
        DEBUG("v>39232 LOW VOLTAGE ON LIPO BLINK LED\r\n", 0);
        return 2;
    }
    
    //return 0 if lipo (3.7-4.2v))
    return 0;

}



void enableArc()
{
    
    
    // ALL OUTPUTS (needed for led)
    // TRISA = 0b11011111;  // RA5 output
    TRISA = 0;  // RA5 output

    TRISA0=1;  // was tut das?! war schon da. siehe oben
    TRISA2=1; // fuer ADC ?    
    
    

}

//called all the time
void loop()
{

    if(checkUSBorLIPO())
    {
        
        
        DEBUG("CHARGING\r\n", 0);
        
        //turn off the arc permanently during charging
        T2CONbits.TMR2ON = 0;
        
        // we run on 5V.. lets blink the led and do nothing else
        do { blinkLed(); }  while(1);

    }
    
    
    
     // when we get here we are not on 5v
  //  DEBUG("MUSIC\r\n", 0);
    
   // while(1){};
   

    enableArc();
   forceArc=1;
    blockingDelay(5000);
    forceArc=0;
    
  //  PR2=0xED;
    
    
    cantinaBand();
  //  imperialMarch();
    
    //tetris();
    while(1){

    }// do nothing hold it forever. };


    // never get here






}









// Main program
int main(int argc, char** argv) {



    setup();
    while(1)
    {
      loop();
    }

    // never get here :(
    return (EXIT_SUCCESS);
}




















// Our global interrupt service routine
static void interrupt isr(void)
{
    // Timer2 interrupt
    // We're basically using Timer 2 to gate Timer 1 in software.
    // postscaler is used to divide the Timer2 frequency by 2
    if(PIR1bits.TMR2IF){
        if(!noGate){
            postscaler^=1;
            if(postscaler){
                gate ^=1;
            }
        } else {
            gate=0;
        }
        PIR1bits.TMR2IF=0;
    }


    // Timer0 interrupt
    // Controls the main PWM frequency, and also times our delays
    if(INTCONbits.TMR0IF){
        if(clockDivider<CLOCK_DIVIDER){
            clockDivider++;
        } else {
            if(genericDelay>0) genericDelay--;
            clockDivider=0;
        }

        // Here's our exceptionally shitty complementary PWM generator
        // You can't simply set LATA4 to the inverse of LATA1 without
        // using an intermediate variable, trust me, it shits itself.
        if(gate || forceArc){
            pinState ^= 1;
            LATA4 = pinState;
            LATA1 = (pinState^1);
        } else {
            LATA4=0;
            LATA1=0;
        }
        INTCONbits.TMR0IF=0;
    }
}

// Generic delay function
void blockingDelay(unsigned int mSecs){
    genericDelay=mSecs;
    while(genericDelay>0);
}

// Note player function
void playNote(unsigned char note, unsigned int duration){
    if(note>0){
        noGate=0;
        PR2=notes[note];
    } else {
        noGate=1;
    }
    blockingDelay(duration);
}



// removed intro - overflo 20.6.2017
void imperialMarch(void){

    // INTRO removed from original sound by podecoet

    playNote(A2,600);
    playNote(SILENCE,500);

    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,400);



    // fixed timing  - overflo 20.6.2017
    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,250);

    playNote(F2,600);
    playNote(SILENCE,250); //was 250

    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,1250);

    
    
    //////////////////////////
    
    
    
    playNote(E3,600);
    playNote(SILENCE,500);

    playNote(E3,500);
    playNote(SILENCE,500);

    playNote(E3,500);
    playNote(SILENCE,500);

    playNote(F3,750); // was 500
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(Ab2,750); // was 250
    playNote(SILENCE,500); // was 500 / 250
    

    playNote(F2,600);     //was 550
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,1250);     //was 1000

    
    
    
    
    
    
    
    //////////
    
    
    
    
    playNote(A3,600);
    playNote(SILENCE,500);

    playNote(A2,500);
    playNote(SILENCE,250);

    playNote(A2,250);
    playNote(A3,500);
    playNote(SILENCE,500);

    playNote(Ab3,500);
    playNote(SILENCE,250);

    playNote(G3,250);
    playNote(Gb3,250);
    playNote(F3,250);
    playNote(Gb3,500);
    playNote(SILENCE,500);

    playNote(Db3,500);
    playNote(F3,750);
    playNote(SILENCE,250);

    playNote(E3,500);
    playNote(SILENCE,250);

    playNote(Eb3,250);
    playNote(D3,250);
    playNote(Db3,250);
    playNote(D3,500);
    playNote(SILENCE,500);

    
    
    
    ///?
    playNote(A2,500);
    playNote(C3,750);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,500);
    playNote(SILENCE,500);

    playNote(F2,500);
    playNote(SILENCE,250);

    playNote(C3,250);
    playNote(A2,750);
    playNote(SILENCE,1250);

}
































void cantinaBand(void){
    
    
    
    
    
    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(E2,250);
    playNote(SILENCE,250);

    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(E2,250);
    playNote(SILENCE,250);

    playNote(B1,250);
    playNote(E2,250);
    playNote(SILENCE,250);

    playNote(B1,400);    
    playNote(SILENCE,100);
    
    playNote(Bb1,250);    
    playNote(B1,250);    
    playNote(SILENCE,250);

    
    
    
    
    playNote(B1,250);    
    playNote(Bb1,250);    
    playNote(B1,250);    
    playNote(A1,350);    
    playNote(SILENCE,150);
    
    playNote(Ab1,250);    
    playNote(A1,250);    
    playNote(SILENCE,250);
    
    playNote(G1,450);    
    playNote(SILENCE,550);

    playNote(E1,400);    
    playNote(SILENCE,650);

    //  block 1 done --ok 
    
    
    
    
  
    
    
    
    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(E2,250);
    playNote(SILENCE,250);

    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(E2,250);
    playNote(SILENCE,250);

    playNote(B1,250);
    playNote(E2,250);
    playNote(SILENCE,250);

    
    
    
    playNote(B1,400);    
    playNote(SILENCE,100);
    


    
    ///////
    
    
    playNote(Bb1,250);    
    playNote(B1,250);    
    playNote(SILENCE,250);
    
    playNote(A1,250);    
    playNote(SILENCE,250);

    playNote(A1,250);    
    playNote(SILENCE,500);
    
      
      
      
     
    
    playNote(A1,250);    
    playNote(A1,250);    

    playNote(SILENCE,250);
    
    playNote(D2,250);
    playNote(SILENCE,250);

    playNote(C2,250);
    playNote(SILENCE,250);

    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(A1,250);
    playNote(SILENCE,250);
    
    
    playNote(B1,250);
    playNote(SILENCE,250);
    
    playNote(E2,250);
    playNote(SILENCE,250);
    
    
    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(E2,250);
    playNote(SILENCE,250);
       
    playNote(B1,250);
    playNote(E2,250);
    playNote(SILENCE,250);
    
    playNote(B1,500);
    

    playNote(Bb1,250);
    playNote(B1,250);
    playNote(SILENCE,250);

    playNote(B1,250);
    playNote(Bb1,250);
    playNote(B1,250);
    playNote(A1,250);
    playNote(SILENCE,250);
    
    playNote(Ab1,250);
    playNote(A1,250);
    playNote(SILENCE,250);
    
    playNote(G1,250);
    playNote(SILENCE,750);
    
    playNote(E1,250);
    playNote(SILENCE,750);
    
    playNote(E1,250);
    playNote(SILENCE,750);

    playNote(G1,250);
    playNote(SILENCE,750);
    
    playNote(B1,250);
    playNote(SILENCE,750);
    
    playNote(D2,250);
    playNote(SILENCE,750);
    
    playNote(F2,250);
    playNote(SILENCE,250);
    
    playNote(E2,250);
    playNote(SILENCE,250);
    
    playNote(Bb1,250);
    playNote(B1,250);
    playNote(SILENCE,250);
    
    playNote(G1,500);
    playNote(SILENCE,250);
    
}

/*
void tetris(void)
{
playNote(E5,320);
playNote(B4,160);
playNote(C5,160);
playNote(D5,160);
playNote(E5,80);
playNote(D5,80);
playNote(C5,160);
playNote(B4,160);
playNote(A4,320);
playNote(A4,160);
playNote(C5,160);
playNote(E5,320);
playNote(D5,160);
playNote(C5,160);
playNote(B4,160);
playNote(E4,160);
playNote(B4,160);
playNote(C5,160);
playNote(D5,320);
playNote(E5,320);
playNote(C5,320);
playNote(A4,320);
playNote(A4,160);
playNote(A3,160);
playNote(B3,160);
playNote(C4,160);
playNote(D4,160);
playNote(D5,320);
playNote(F5,160);
playNote(A5,160);
playNote(A5,80);
playNote(A5,80);
playNote(G5,160);
playNote(F5,160);
playNote(E5,480);
playNote(C5,160);
playNote(E5,160);
playNote(F5,80);
playNote(E5,80);
playNote(D5,160);
playNote(C5,160);
playNote(B4,320);
playNote(B4,160);
playNote(C5,160);
playNote(D5,320);
playNote(E5,320);
playNote(C5,320);
playNote(A4,320);
playNote(A4,640);
playNote(E4,160);
playNote(C4,160);
playNote(A3,160);
playNote(C4,160);
playNote(E4,160);
playNote(C4,160);
playNote(A3,160);
playNote(C4,160);
playNote(D4,160);
playNote(B3,160);
playNote(GS3,160);
playNote(B3,160);
playNote(D4,160);
playNote(B3,160);
playNote(GS3,160);
playNote(B3,160);
playNote(C4,160);
playNote(A3,160);
playNote(E3,160);
playNote(A3,160);
playNote(C4,160);
playNote(A3,160);
playNote(E3,160);
playNote(A3,160);
playNote(B3,160);
playNote(GS3,160);
playNote(E3,160);
playNote(GS3,160);
playNote(B3,640);
playNote(E4,160);
playNote(C4,160);
playNote(A3,160);
playNote(C4,160);
playNote(E4,160);
playNote(C4,160);
playNote(A3,160);
playNote(C4,160);
playNote(D4,160);
playNote(B3,160);
playNote(GS3,160);
playNote(B3,160);

}
 */
