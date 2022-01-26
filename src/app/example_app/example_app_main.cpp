/****************************************************************************
 *   Aug 3 12:17:11 2020
 *   Copyright  2020  Dirk Brosswick
 *   Email: dirk.brosswick@googlemail.com
 ****************************************************************************/
 
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

#include <HardwareSerial.h>  //$$$$$$$$$$$$$$$$$
//#include <SoftwareSerial.h> //CURT

#define hardserial_TX      (GPIO_NUM_36) //36 
#define hardserial_RX      (GPIO_NUM_39) //39 

#define DUMMY_TX      (GPIO_NUM_15) //36 
#define DUMMY_RX      (GPIO_NUM_2) //39 

static const uint32_t GRDBaud = 115200;

//const char startDelimiter = '<';
//const char endDelimiter   = '>';

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void recvWithStartEndMarkers();
void showNewData();


//-----------------------------------------------
lv_obj_t *example_app_main_tile = NULL;

lv_task_t * _example_app_task_250;

lv_task_t * _example_app_task_1000;
lv_task_t * _example_app_play_sound_task = nullptr;

LV_IMG_DECLARE(refresh_32px);
LV_FONT_DECLARE(Ubuntu_72px);

static void exit_example_app_main_event_cb( lv_obj_t * obj, lv_event_t event );
static void enter_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event );

void example_app_task_250( lv_task_t * task ); //hardware serial

void example_app_task_1000( lv_task_t * task );

static void example_app_play_sound_task( lv_task_t * task );

void example_app_main_setup( uint32_t tile_num ) {



    //HARDWARE SERIAL
    Serial2.begin( GRDBaud, SERIAL_8N1, hardserial_RX, hardserial_TX );
    delay(150);



    example_app_main_tile = mainbar_get_tile_obj( tile_num );

    lv_obj_t * exit_btn = wf_add_exit_button( example_app_main_tile, exit_example_app_main_event_cb );
    lv_obj_align(exit_btn, example_app_main_tile, LV_ALIGN_IN_BOTTOM_LEFT, THEME_ICON_PADDING, -THEME_ICON_PADDING );

    lv_obj_t * setup_btn = wf_add_setup_button( example_app_main_tile, enter_example_app_setup_event_cb );
    lv_obj_align(setup_btn, example_app_main_tile, LV_ALIGN_IN_BOTTOM_RIGHT, -THEME_ICON_PADDING, -THEME_ICON_PADDING );


    //HARDWARE SERIAL TASK
   // _example_app_task_250 = lv_task_create( example_app_task_250, 250, LV_TASK_PRIO_HIGH, NULL ); //write serial
    _example_app_task_250 = lv_task_create( example_app_task_250, 250, LV_TASK_PRIO_MID, NULL ); //write serial

    // create an task that runs every 10 secounds
    _example_app_task_1000 = lv_task_create( example_app_task_1000, 1000, LV_TASK_PRIO_MID, NULL ); //read serial

}

static void enter_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_CLICKED ):       mainbar_jump_to_tilenumber( example_app_get_app_setup_tile_num(), LV_ANIM_ON );
                                        statusbar_hide( true );
                                        log_i("----------------- CURT -------- enter_example_app_setup_event_cb");
                                        break;
    }
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
    log_i("----------------- CURT -------- example_app_play_sound_task");
 //   sound_play_progmem_wav( test_c_mouth_wav, test_c_mouth_wav_len ); 
    //sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    //motor_vibe(100); 
}


void example_app_task_250( lv_task_t * task ) {
    recvWithStartEndMarkers();
    showNewData();
}

void example_app_task_1000( lv_task_t * task ) {
    log_i("----------------- CURT -------- example_app_task_1000 HARDWARE SERIAL WRITE");


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

}

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