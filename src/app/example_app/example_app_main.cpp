/****************************************************************************
 *   Aug 3 12:17:11 2020
 *   Copyright  2020  Dirk Brosswick
 *   Email: dirk.brosswick@googlemail.com
 ****************************************************************************/


/*
HACKED GPIO
39 (in only)   purple
36 (in only)   yellow
12             green
13             white   TWATCH_2020_IR_PIN

vbat           red
GND            black
*/
 
/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include "config.h"
#include "hardware/powermgm.h" //CURT
//#include "callback.h" //CURT

#include "example_app.h"
#include "example_app_main.h"

#include "gui/mainbar/app_tile/app_tile.h"
#include "gui/mainbar/main_tile/main_tile.h"
#include "gui/mainbar/mainbar.h"
#include "gui/statusbar.h"

#include "gui/sound/piep.h" //CURT
#include "gui/sound/test_c_mouth.h" //CURT

#include "gui/widget_factory.h"
#include "gui/widget_styles.h"

#include "hardware/sound.h" //CURT
#include "hardware/motor.h" //CURT

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include <HardwareSerial.h> 

//USE IR GPIO FOR OUTPUT
//bool testIrpinToggle = true;

/*
#define hardserial_RX      (GPIO_NUM_39) //36 
#define hardserial_TX      (GPIO_NUM_13) //39 

//#define DUMMY_TX      (GPIO_NUM_15) //36 
//#define DUMMY_RX      (GPIO_NUM_2) //39 

static const uint32_t GRDBaud = 9600; //115200;

//HardwareSerial Serial2(2); 

unsigned char buf_recive[1];
unsigned char buf_transmit[2] = {0x42, 0x00};


//const char startDelimiter = '<';
//const char endDelimiter   = '>';
char startDelimiter = '<';
char endDelimiter   = '>';

const byte numChars = 32;
//char receivedChars[numChars];   // an array to store the received data
//char tempChars[numChars];        // temporary array for use when parsing

//boolean newData = false;

//void recvWithStartEndMarkers();
//void showNewData();
*/

//-----------------------------------------------
lv_obj_t *example_app_main_tile = NULL;

lv_task_t * _example_app_task_5000;

lv_task_t * _example_app_task_10000;
lv_task_t * _example_app_play_sound_task = nullptr;

LV_IMG_DECLARE(refresh_32px);
LV_FONT_DECLARE(Ubuntu_72px);

static void exit_example_app_main_event_cb( lv_obj_t * obj, lv_event_t event );
static void enter_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event );

void example_app_task_5000( lv_task_t * task ); //hardware serial

void example_app_task_10000( lv_task_t * task );

static void example_app_play_sound_task( lv_task_t * task );

void example_app_main_setup( uint32_t tile_num ) {

//USE IR GPIO FOR OUTPUT
//pinMode(TWATCH_2020_IR_PIN, OUTPUT);
//digitalWrite(TWATCH_2020_IR_PIN, LOW); //see what happen


    //HARDWARE SERIAL
    //Serial2.begin( GRDBaud, SERIAL_8N1, hardserial_RX, hardserial_TX );
    //    Serial2.begin( 9600, SERIAL_8N1, hardserial_RX, hardserial_TX );
  //  delay(250);



    example_app_main_tile = mainbar_get_tile_obj( tile_num );

    lv_obj_t * exit_btn = wf_add_exit_button( example_app_main_tile, exit_example_app_main_event_cb );
    lv_obj_align(exit_btn, example_app_main_tile, LV_ALIGN_IN_BOTTOM_LEFT, THEME_ICON_PADDING, -THEME_ICON_PADDING );

    lv_obj_t * setup_btn = wf_add_setup_button( example_app_main_tile, enter_example_app_setup_event_cb );
    lv_obj_align(setup_btn, example_app_main_tile, LV_ALIGN_IN_BOTTOM_RIGHT, -THEME_ICON_PADDING, -THEME_ICON_PADDING );




}

static void enter_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_CLICKED ):       mainbar_jump_to_tilenumber( example_app_get_app_setup_tile_num(), LV_ANIM_ON );
                                        statusbar_hide( true );
                                        log_i("----------------- CURT -------- enter_example_app_setup_event_cb");
                                        break;
    }

        //HARDWARE SERIAL TASK
   // _example_app_task_250 = lv_task_create( example_app_task_250, 250, LV_TASK_PRIO_HIGH, NULL ); //write serial
    _example_app_task_5000 = lv_task_create( example_app_task_5000, 5000, LV_TASK_PRIO_MID, NULL ); //write serial

    // create an task that runs every 1 secounds
    _example_app_task_10000 = lv_task_create( example_app_task_10000, 10000, LV_TASK_PRIO_MID, NULL ); //read serial
}

static void exit_example_app_main_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_CLICKED ):       
            log_i("----------------- CURT -------- exit_example_app_main_event_cb");
            mainbar_jump_back();
            break;
    }


}

static void example_app_play_sound_task( lv_task_t * task )
{
  //  log_i("----------------- CURT -------- example_app_play_sound_task");
 //   sound_play_progmem_wav( test_c_mouth_wav, test_c_mouth_wav_len ); 
    //sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    //motor_vibe(100); 
}


void example_app_task_5000( lv_task_t * task ) {


//============================================================== TOGGLE IR PIN

//    if(testIrpinToggle){
//        digitalWrite(TWATCH_2020_IR_PIN, HIGH);
//        testIrpinToggle = false;
//    } else {
//        digitalWrite(TWATCH_2020_IR_PIN, LOW);
//        testIrpinToggle = true;
//    }


 //   recvWithStartEndMarkers();
 //   showNewData();
    //    log_i("----- GRD ------ example_app_task_5000 ** HARDWARE SERIAL READ loop test");

    //clear char array
   // char foo[5];
    //memset (receivedChars, '.', sizeof (receivedChars) - 1);
    //receivedChars [sizeof (receivedChars) - 1] = 0;
     //   memset(receivedChars, 0, sizeof(receivedChars));   //clear array
/*
    //MUST CONNECT TX and RX FOR TEST
    if ( Serial2 ) {
        char outByte1 = 'a';
        char outByte2 = 'b';
        char outByte3 = 'c';
        char inByte1;

        char receivedChars[numChars];

        //static boolean recvInProgress = false;
        static byte ndx = 0;
        //char startMarker = '<';
        //char endMarker = '>';
        char rc;

        //send test data
      //  Serial2.write(outByte1);
      //  Serial2.write(outByte2);
      //  Serial2.write(outByte3);
     //   delayMicroseconds(50);

      //  if(Serial2.available() < 33){
          //  for (int i = 0; i < 5; i++){
         //       Serial2.write(startDelimiter);    
                //Serial2.write(rand ());    // send the number
         //       Serial2.write(outByte1);
          //      Serial2.write(endDelimiter);  
          //  }  // end of for
     //   }

      //  delay(1);
      //  Serial2.flush();
        //Serial2.write(50);
        Serial2.write(buf_transmit, 2);
        delay(50);

  // detect first byte of recieve message
  if (Serial2.find(0x42))
  { //start to read when detect 0x42
    Serial2.readBytes(buf_recive, 1);
    Serial.println(buf_recive[0]);
    log_i("%hhx\n", buf_recive[0]);
  }
*/
        //check for serial data and read receivedChars
/*
        if ( Serial2.available() > 0) {
            delay(5);
            log_i("Serial2 buffer data (%d bytes)", Serial2.available() ); //amount of data
            delay(1);

            while ( Serial2.available() > 0){  //read one char at a time

                rc = (char)Serial2.read(); //read softserial

                if(ndx < numChars){
                    receivedChars[ndx] = rc;
                  //  log_i("Serial2: %c", inByte1 );

               //     delayMicroseconds(50);
                    ndx = ndx + 1;
                }
            }
        }


        for(int i=0; i<ndx; i++){
            log_i("RC: %hhx", receivedChars[i] );
        }
*/
/*
        //EXTRA SAFETY FLUSH
  delay(5);
        while ( Serial2.available() > 0){
            char dummy = (char)Serial2.read();
        }
        Serial2.flush();

    }
    */
}


void example_app_task_10000( lv_task_t * task ) {
 //   log_i("----------------- CURT -------- example_app_task_10000");
/*

    //MUST CONNECT TX and RX FOR TEST
    if ( Serial2 ) {
        char incomByte1 = 'a';
        char incomByte2 = 'b';
        char incomByte3 = 'c';

        //send test data


        Serial2.write('<');    
        Serial2.write(incomByte1);
        Serial2.write(incomByte2);
        Serial2.write(incomByte3);
        Serial2.write('>');  
    }
*/

}

/*
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial2.available() > 0 && newData == false) {
        rc = Serial2.read();
        log_i("read: %c", rc);

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        log_i("This just in ... %s", receivedChars);
        newData = false;
    }
}
*/