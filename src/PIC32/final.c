//https://people.ece.cornell.edu/land/courses/ece4760/PIC32/PLIB_examples/plib_examples/uart/uart_interrupt/source/uart_interrupt.c
/*
 * File:        TFT, keypad, DAC, LED, PORT EXPANDER test
 *              With serial interface to PuTTY console
 * Author:      Bruce Land
 * For use with Sean Carroll's Big Board
 * http://people.ece.cornell.edu/land/courses/ece4760/PIC32/target_board.html
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
//  TEST OLD CODE WITH NEW THREADS
//#include "config_1_2_3.h"
#include "config_1_3_2.h"
// threading library
//#include "pt_cornell_1_2_3.h"
#include "pt_cornell_1_3_2.h"
// yup, the expander
#include "port_expander_brl4.h"

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
#include <string.h>
////////////////////////////////////

// lock out timer 2 interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)

#define manualMode 0
#define autoMode 1
#define menuMode 2
int currMode = autoMode;
int curr_x, curr_y;

#define joystick_x 519
#define joystick_y 518
////////////////////////////////////

/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// string buffer
char buffer[60];
char ip[15];

////////////////////////////////////
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 4 ; // 10 MHz max speed for port expander!!

// === print a line on TFT =====================================================
// print a line on the TFT
// string buffer
char buffer[60];
void printLine(int line_number, char* print_buffer, short text_color, short back_color){
  // line number 0 to 31 
  /// !!! assumes tft_setRotation(0);
  // print_buffer is the string to print
  int v_pos;
  v_pos = line_number * 10 ;
  // erase the pixels
  tft_fillRoundRect(0, v_pos, 319, 8, 1, back_color);// x,y,w,h,radius,color
  tft_setTextColor(text_color); 
  tft_setCursor(0, v_pos);
  tft_setTextSize(1);
  tft_writeString(print_buffer);
}

void printLine2(int line_number, char* print_buffer, short text_color, short back_color){
  // line number 0 to 31 
  /// !!! assumes tft_setRotation(0);
  // print_buffer is the string to print
  int v_pos;
  v_pos = line_number * 20 ;
  // erase the pixels
  tft_fillRoundRect(0, v_pos, 319, 16, 1, back_color);// x,y,w,h,radius,color
  tft_setTextColor(text_color); 
  tft_setCursor(0, v_pos);
  tft_setTextSize(2);
  tft_writeString(print_buffer);
}

// Predefined colors definitions (from tft_master.h)
#define	ILI9340_BLACK   0x0000
#define	ILI9340_BLUE    0x001F
#define	ILI9340_RED     0xF800
#define	ILI9340_GREEN   0x07E0
#define ILI9340_CYAN    0x07FF
#define ILI9340_MAGENTA 0xF81F
#define ILI9340_YELLOW  0xFFE0
#define ILI9340_WHITE   0xFFFF

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_serial, pt_manual, pt_test, pt_menu ;
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output ;

// system 1 second interval tick
int sys_time_seconds ;
void delayCycle(int cycle){
  int i;
  for (i=0;i<cycle;i++) __asm__("nop");
}
const char* mode_text[] = {"manual mode","auto mode","menu mode"};
// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
  PT_BEGIN(pt);
  int i;
  while(1) {
    // yield time 1 second
    PT_YIELD_TIME_msec(1000) ;
    sys_time_seconds++ ;
    if (currMode != menuMode){
      sprintf(buffer,"Time=%d, mode=%s", sys_time_seconds,mode_text[currMode]);
      printLine2(0, buffer, ILI9340_WHITE, ILI9340_BLACK);
      sprintf(buffer,"%d,%d", curr_x,curr_y);
      printLine2(1, buffer, ILI9340_WHITE, ILI9340_BLACK);
    }
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // timer thread

int x_adc,y_adc;
// default down
int penDown = 1;
void controlPen(int value){
  if (value && penDown == 0){
    SetDCOC3PWM(110);
    delayCycle(1050000);
    SetDCOC3PWM(0);
    penDown = 1;
  }else if (value == 0 && penDown == 1){
    SetDCOC3PWM(360);
    delayCycle(1000000);
    SetDCOC3PWM(0);
    penDown = 0;
  }
}
int print_speed = 5000;
void updateCoor(int x, int y){
  int i;
  int diff_x = x-curr_x;
  int diff_y = y-curr_y;
  if (diff_x != 0 && diff_y != 0){ // control both motors
    if (diff_x < 0) { mPORTBSetBits(BIT_4);diff_x=-diff_x;}
    else mPORTBClearBits(BIT_4);
    if (diff_y < 0) { mPORTBClearBits(BIT_0);diff_y=-diff_y;}
    else mPORTBSetBits(BIT_0);
    // calculate delay to limit maximum speed specified by print_speed
    int delay = diff_x > diff_y ? (float)print_speed/diff_y : (float)print_speed/diff_x;
    for (i = 0; i < diff_x*diff_y; i++){ 
      if (i % diff_x == 0) mPORTAToggleBits(BIT_2);
      if (i % diff_y == 0) mPORTAToggleBits(BIT_0);
      delayCycle(delay);
    }
  }else if (diff_x != 0){
    if (diff_x < 0) { mPORTBSetBits(BIT_4);diff_x=-diff_x;}
    else mPORTBClearBits(BIT_4);
    for (i = 0;i < diff_x; i++){ 
      mPORTAToggleBits(BIT_0);
      delayCycle(print_speed);
    }
  }else if (diff_y != 0){
    if (diff_y < 0) { mPORTBClearBits(BIT_0);diff_y=-diff_y;}
    else mPORTBSetBits(BIT_0);
    for (i=0;i<diff_y;i++){ 
      mPORTAToggleBits(BIT_2);
      delayCycle(print_speed);
    }
  }
  // update coordinates
  curr_x = x;
  curr_y = y;
}

static PT_THREAD (protothread_manual(struct pt *pt))
{
  PT_BEGIN(pt);
  while(1) {
    if (currMode != manualMode){
      PT_YIELD_TIME_msec(100);
      continue;
    }
    PT_YIELD_TIME_msec(1);
    x_adc = ReadADC10(0);   // read AN1
    y_adc = 1023-ReadADC10(1);   // read AN11
    updateCoor(curr_x+10*((x_adc-joystick_x)/30),curr_y+10*((y_adc-joystick_y)/30));
  } // END WHILE(1)
  PT_END(pt);
} // manual thread

#define NOT_PUSHED 0
#define MAYBE_PUSHED 1
#define PUSHED 2 
#define MAYBE_NOT_PUSHED 3
short curr_debounce; 

#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;

char* level_1_text[]={"1. Switch Mode","2. Pen Control","3. Print Speed","4. View RPi IP", "5. Calibrate Base", "6. Calibrate Pen","7. Exit"};
char* level_2_1_text[]={"1. Auto Mode","2. Manual Mode"};
char* level_2_2_text[]={"1. Down","2. Up"};
char* level_2_3_text[]={"1. Fast","2. Medium","3. Slow"};
char* level_2_4_text[]={"Press Button To Exit"};
char* level_2_5_text[]={"When pen at top left region, press button"};
char* level_2_6_text[]={"When pen just touches the plate, press button"};
int level2_count[] = {2,2,3,1,1,1};
int menu_level = 0;
int option = 0;
int lv1_option;
char** level_2_ref [] = {level_2_1_text,level_2_2_text,level_2_3_text,level_2_4_text,level_2_5_text,level_2_6_text};
static PT_THREAD (protothread_menu(struct pt *pt))
{
  PT_BEGIN(pt);
  int i;
  // set B3 for button
  EnablePullUpB(BIT_3);
  mPORTBSetPinsDigitalIn(BIT_3);
  while(1) {
    PT_YIELD_TIME_msec(60);
    unsigned char state = mPORTBReadBits(BIT_3);
    int pressed = 0;
    switch (curr_debounce){
      case NOT_PUSHED:
        // if button pushed
        if (state == 0) curr_debounce = MAYBE_PUSHED;
        break;
      case MAYBE_PUSHED:
        // if button pushed again
        if (state == 0) curr_debounce = PUSHED;
        else curr_debounce = NOT_PUSHED;
        break;
      case PUSHED:
        // one-time signal, transition away
        pressed = 1;
        curr_debounce = MAYBE_NOT_PUSHED;
        break;
      case MAYBE_NOT_PUSHED:
        // if not pushed
        if (state != 0) curr_debounce = NOT_PUSHED;
        break;
    }

    switch (menu_level){
      case 0:
        if (pressed) {
          // button pressed, switch to menu mode
          currMode = menuMode;
          menu_level = 1;
          tft_fillScreen(ILI9340_BLACK);
          // print all options
          for (i = 0; i < 7; i++) printLine2(i, level_1_text[i], ILI9340_WHITE, ILI9340_BLACK);
          option = 0;
        }
        break;
      case 1:
        if (pressed) {
          if (option == 6){
            // exit menu
            menu_level = 0;
            tft_fillScreen(ILI9340_BLACK);
            currMode = autoMode;
          }else{
            // go to next level
            menu_level = 2;
            tft_fillScreen(ILI9340_BLACK);
            for (i = 0; i < level2_count[option]; i++) printLine2(i, level_2_ref[option][i], ILI9340_WHITE, ILI9340_BLACK);
            lv1_option = option;
          }
          option = 0;
        }else{
          // if not pressed, browse menu
          x_adc = 1023-ReadADC10(1);   // read the result of channel 9 conversion from the idle buffer
          if (x_adc > 1000){
            if (option < 6){
              // highlight currently selected option
              option++;
              printLine2(option-1, level_1_text[option-1], ILI9340_WHITE, ILI9340_BLACK);
              printLine2(option, level_1_text[option], ILI9340_BLACK, ILI9340_WHITE);
            }
          }else if (x_adc < 23){
            if (option > 0){
              // highlight currently selected option
              option--;
              printLine2(option, level_1_text[option], ILI9340_BLACK, ILI9340_WHITE);
              printLine2(option+1, level_1_text[option+1], ILI9340_WHITE, ILI9340_BLACK);
            }
          }
        }
        break;
      case 2:
        if (pressed) {
          menu_level = 0;
          currMode = autoMode;
          tft_fillScreen(ILI9340_BLACK);
          switch (lv1_option){
            case 0:
              if (option == 0) currMode = autoMode;
              if (option == 1) currMode = manualMode;
              break;
            case 1:
              if (option == 0) controlPen(1);
              if (option == 1) controlPen(0);
              break;
            case 2:
              if (option == 0) print_speed = 5000;
              if (option == 1) print_speed = 10000;
              if (option == 2) print_speed = 15000;
              break;
            case 3:
              break;
            case 4:
              curr_x = 0;
              curr_y = 0;
              break;
            case 5:
              controlPen(0);
            break;
          }
        }else{
          x_adc = ReadADC10(0);   // read AN1
          y_adc = 1023-ReadADC10(1);   // read AN11
          switch (lv1_option){
          case 0:
          case 1:
          case 2:
            // no action, browsing
            if (y_adc > 1000){
              if (option < level2_count[lv1_option]-1){
                option++;
                printLine2(option-1, level_2_ref[lv1_option][option-1], ILI9340_WHITE, ILI9340_BLACK);
                printLine2(option, level_2_ref[lv1_option][option], ILI9340_BLACK, ILI9340_WHITE);
              }
            }else if (y_adc < 23){
              if (option > 0){
                option--;
                printLine2(option, level_2_ref[lv1_option][option], ILI9340_BLACK, ILI9340_WHITE);
                printLine2(option+1, level_2_ref[lv1_option][option+1], ILI9340_WHITE, ILI9340_BLACK);
              }
            }
            break;
          case 3:
            sprintf(buffer, "RPi IP = %s",ip);
            printLine2(1, buffer, ILI9340_WHITE, ILI9340_BLACK);
            break;
          case 4:
            updateCoor(curr_x+10*((x_adc-joystick_x)/30),curr_y+10*((y_adc-joystick_y)/30));
            break;
          case 5:
            controlPen(1);
            break;      
          }
        }
        break;
    }
  } // END WHILE(1)
  PT_END(pt);
} // menu thread

//=== Serial terminal thread =================================================

static PT_THREAD (protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  static char cmd[30],cmd2[30];
  static int value, value2;
  static int n = 0;
  while(1) {
    // send last message as ACK
    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
    //spawn a thread to handle terminal input
    PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input) );
    // received a command
    if(PT_timeout==0) sscanf(PT_term_buffer, "%s %d %s %d\n\r", cmd, &value, cmd2, &value2);
    // no commands
    else cmd[0] = 0;     
    switch(cmd[0]){
      case 'I': 
        sscanf(PT_term_buffer, "%s %s\n\r", cmd, ip);
        break;
      case 'D': 
        // only execute commands in auto mode
        if (currMode != autoMode){
        PT_YIELD_TIME_msec(100);
        continue;
        }
        controlPen(value);
        break;
      case 'X': 
        if (currMode != autoMode){
        PT_YIELD_TIME_msec(100);
        continue;
        }
        // mirrored because coordinates meant for pen, but we move plate
        updateCoor(-value,-value2);
        break; 
    }
    if (cmd[0] != 0){
      // construct ACK message
      strcpy(PT_send_buffer,PT_term_buffer);
      // spawn an ACK thread
      PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
    }
    // never exit while
  } // END WHILE(1)
  PT_END(pt);
} // serial thread

// === Main  ======================================================
void main(void) {
  ANSELA = 0; ANSELB = 0; 
  // === ADC initial set up =======
  // configure and enable the ADC
  CloseADC10();	// ensure the ADC is off before setting the configuration

  // define setup parameters for OpenADC10
  // Turn module on | output in integer | trigger mode auto | enable  autosample
  #define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

  // define setup parameters for OpenADC10
  // ADC ref external | disable offset test | enable scan mode | perform 2 samples | use one buffer | use MUXA mode
  // note: to read X number of pins you must set ADC_SAMPLES_PER_INT_X
  #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF

  // define setup parameters for OpenADC10
  // use ADC internal clock | set sample time
  #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

  // define setup parameters for OpenADC10
  // set AN1 and AN11
  #define PARAM4 ENABLE_AN1_ANA | ENABLE_AN11_ANA // AN11: pin 24

  // define setup parameters for OpenADC10
  // do not assign channels to scan
  #define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

  // use ground as neg ref for A 
  SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); // use ground as the negative reference
  OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameter define above

  EnableADC10(); // Enable the ADC

  // timer interrupt //////////////////////////
  // Set up timer2 on,  interrupts, internal clock, prescalar 256, period 3125
  // 50Hz
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 3125);

  mT2ClearIntFlag(); // and clear the interrupt flag

  // control CS for DAC
  // mPORTBSetPinsDigitalOut(BIT_4);
  // mPORTBSetBits(BIT_4);
  // SCK2 is pin 26 
  // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
  PPSOutput(2, RPB5, SDO2);
  // 16 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripherial, you will need to match these
  // NOTE!! IF you are using the port expander THEN
  // -- clk divider must be set to 4 for 10 MHz
  SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 4);
  // end DAC setup
  OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , 0, 0); //
  // OC3 is PPS group 4, map to RPB9 (pin 18)
  PPSOutput(4, RPB9, OC3);

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_serial);
  PT_INIT(&pt_manual);
  PT_INIT(&pt_test);
  PT_INIT(&pt_menu);
  // motor control pins
  // motor 1 step
  mPORTASetBits(BIT_0 );
  mPORTASetPinsDigitalOut(BIT_0 ); 
  // motor 1 dir
  mPORTBSetBits(BIT_4 ); 
  mPORTBSetPinsDigitalOut(BIT_4 );    
  // motor 2 step
  mPORTASetBits(BIT_2 );
  mPORTASetPinsDigitalOut(BIT_2 );  
  // motor 2 dir
  mPORTBSetBits(BIT_0 ); 
  mPORTBSetPinsDigitalOut(BIT_0 ); 

  // init the display
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //320x240 horizontal display
  tft_setRotation(1);
  // construct first ACK message
  sprintf(PT_send_buffer,"X 0 Y 0");
  // round-robin scheduler for threads
  while (1){
    PT_SCHEDULE(protothread_timer(&pt_timer));
    PT_SCHEDULE(protothread_serial(&pt_serial));
    PT_SCHEDULE(protothread_manual(&pt_manual));
    PT_SCHEDULE(protothread_menu(&pt_menu));
  }
} // main

// === end  ======================================================
