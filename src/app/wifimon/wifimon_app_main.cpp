/****************************************************************************
 *   linuxthor 2020
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

//#include "hardware/wifictl.h"
#include "hardware/display.h"


#include "wifimon_app.h"
#include "wifimon_app_main.h"

#include "gui/mainbar/app_tile/app_tile.h"
#include "gui/mainbar/main_tile/main_tile.h"
#include "gui/mainbar/mainbar.h"
#include "gui/statusbar.h"
#include "gui/keyboard.h"
#include "gui/widget_styles.h"
#include "gui/widget_factory.h"

#include "hardware/sound.h"
//#include "gui/sound/piep.h"
//#include "gui/sound/test_c_mouth.h"
#include "hardware/motor.h"
#include "hardware/powermgm.h"
//#include "app/alarm_clock/alarm_in_progress.h"
#include <HardwareSerial.h> 

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <lwip/sockets.h>
//#include "esp_wifi.h"

//GLOBAL VAR FOR SERIAL SENSOR DATA
const byte numChars = 128;
int numCharsInt = 128;
char recievedSerial1[numChars];
//char recievedSerial2[numChars];

float dataMLX15[3] = {0.0, 0.0, 0.0};

uint8_t level = 0, channel = 1;
int data = 0, mgmt = 0, misc = 0; 

lv_obj_t *wifimon_app_main_tile = NULL;
lv_obj_t *chart = NULL;
lv_obj_t *channel_select = NULL; 
lv_chart_series_t *ser1 = NULL;
lv_chart_series_t *ser2 = NULL;
lv_chart_series_t *ser3 = NULL;
lv_task_t *_wifimon_app_task = NULL;

//CUSTOM SERIAL READ TASK
//lv_task_t *_serial_read_sensors_app_task = NULL;

int wifimon_display_timeout = 0;

//lv_task_t * _Play_Target_Sound_task = nullptr;  //curt add

LV_IMG_DECLARE(exit_dark_48px);
LV_IMG_DECLARE(wifimon_app_32px);
LV_FONT_DECLARE(Ubuntu_72px);

static void exit_wifimon_app_main_event_cb( lv_obj_t * obj, lv_event_t event );
static void wifimon_sniffer_set_channel( uint8_t channel );
static void wifimon_app_task( lv_task_t * task );

//CUSTOM SERIAL READ TASK
//static void serial_read_sensors_app_task( lv_task_t * task );

static void wifimon_activate_cb( void );
static void wifimon_hibernate_cb( void );

//static void Play_Target_Sound_task( lv_task_t * task );   //curt add
//static void wifimon_test_play_sound( void );


static void wifimon_sniffer_set_channel( uint8_t channel ) {
 //   esp_wifi_set_channel( channel, WIFI_SECOND_CHAN_NONE );
  //  log_i("set wifi channel: %d", channel );
}

static void wifimon_channel_select_event_handler( lv_obj_t * obj, lv_event_t event ) {
//  log_i("-----------void wifimon_channel_select_event_handler");
    switch( event ) {
        case LV_EVENT_VALUE_CHANGED: {
            char buf[32];
            lv_roller_get_selected_str( obj, buf, sizeof( buf ) );
            wifimon_sniffer_set_channel( atoi(buf) );
            break;
        }
    }
}

 //curt add
/*
static void Play_Target_Sound_task( lv_task_t * task ){   //curt add
        sound_set_enabled_config( true );
    sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    sound_play_progmem_wav( piep_wav, piep_wav_len ); 
    motor_vibe(100); 
}

static void wifimon_test_play_sound( void ) {
        sound_set_enabled_config( true );
    sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    sound_play_progmem_wav(test_c_mouth_wav, test_c_mouth_wav_len);
}
*/



/*******************************************************************************************************
********************************************************************************************************
********************************************************************************************************
                              ********* SETUP **********
                              ********* SETUP **********
                              ********* SETUP **********
                              ********* SETUP **********
                              ********* SETUP **********
********************************************************************************************************
********************************************************************************************************
*******************************************************************************************************/
void wifimon_app_main_setup( uint32_t tile_num ) {

    log_i("----------------- CURT -------- SETUP SETUP SETUP   wifimon_app_main_setup");

    Serial1.begin( 14400, SERIAL_8N1, 39, 1 ); //1 in unconnected... I think
   // Serial1.begin( 19200, SERIAL_8N1, 39, 13 );
    delay(150);
    //Serial2.begin( 19200, SERIAL_8N1, 36, 12 );
   // Serial2.begin( 14400, SERIAL_8N1, 36, 1 ); //1 in unconnected... I think
    //delay(150);

    pinMode(13, OUTPUT); //gpio 13 originally IR LED, now sensor power enable
    digitalWrite(13, LOW);

    pinMode(12, OUTPUT); //gpio 13 originally IR LED, now sensor power enable
    digitalWrite(12, LOW);

    delay(150);


    wifimon_app_main_tile = mainbar_get_tile_obj( tile_num );
    /**
     * add chart widget
     */
    chart = lv_chart_create( wifimon_app_main_tile, NULL );
    lv_obj_set_size( chart, lv_disp_get_hor_res( NULL ), lv_disp_get_ver_res( NULL ) - THEME_ICON_SIZE );
    lv_obj_align( chart, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0 );
    lv_chart_set_type( chart, LV_CHART_TYPE_LINE );  
    lv_chart_set_point_count( chart, 32 );
    lv_obj_set_style_local_bg_opa( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_OPA_50 );
    lv_obj_set_style_local_bg_grad_dir( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_GRAD_DIR_VER );
    lv_obj_set_style_local_bg_main_stop( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 255 );
    lv_obj_set_style_local_bg_grad_stop( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 0 );
    /**
     * add chart series
     */
    ser1 = lv_chart_add_series( chart, LV_COLOR_RED );
    ser2 = lv_chart_add_series( chart, LV_COLOR_GREEN );
    ser3 = lv_chart_add_series( chart, LV_COLOR_YELLOW );
    /**
     * add exit button
     */
    lv_obj_t * exit_btn = wf_add_exit_button( wifimon_app_main_tile, exit_wifimon_app_main_event_cb );
    lv_obj_align( exit_btn, wifimon_app_main_tile, LV_ALIGN_IN_BOTTOM_LEFT, THEME_ICON_PADDING, -THEME_ICON_PADDING );
    /**
     * add channel select roller
     */
    channel_select = lv_roller_create(wifimon_app_main_tile, NULL);
    lv_roller_set_options( channel_select, "IMU\nTherm\nLidar\nIR\nTemp\nLight\nRGB\nAcc\nGyro\nDist\nProx\nAct\nResp", LV_ROLLER_MODE_INIFINITE );
    lv_roller_set_visible_row_count( channel_select, 5 );
    lv_obj_align( channel_select, NULL, LV_ALIGN_IN_TOP_LEFT, THEME_ICON_PADDING, THEME_ICON_PADDING );
    lv_obj_set_event_cb( channel_select, wifimon_channel_select_event_handler );
    /**
     * add chart series label
     */
    lv_obj_t * chart_series_label = lv_label_create( wifimon_app_main_tile, NULL );
    lv_label_set_long_mode( chart_series_label, LV_LABEL_LONG_BREAK );
    lv_label_set_recolor( chart_series_label, true );
    lv_label_set_align( chart_series_label, LV_LABEL_ALIGN_RIGHT );       
    lv_label_set_text( chart_series_label, "#ffff00 - xxxx#\n#ff0000 - yyyy#\n#11ff00 - zzzz#"); 
    lv_obj_set_width( chart_series_label, 70 );
    lv_obj_align( chart_series_label, NULL, LV_ALIGN_IN_TOP_RIGHT, -THEME_ICON_PADDING, THEME_ICON_PADDING );

    mainbar_add_tile_activate_cb( tile_num, wifimon_activate_cb );
    mainbar_add_tile_hibernate_cb( tile_num, wifimon_hibernate_cb );
}

static void exit_wifimon_app_main_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_CLICKED ):     mainbar_jump_back();
                                      break;
    }
}

static void wifimon_hibernate_cb( void ) {
    log_i("----------------- CURT -------- wifimon_hibernate_cb");
    if(_wifimon_app_task != NULL) {
        lv_task_del(_wifimon_app_task);
        _wifimon_app_task = NULL;
    }  

// ======================================= TURN SENSORS OFF ====================================
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    delay(300);

// EXIT CUSTOM SERIAL READ TASK
  //  if(_serial_read_sensors_app_task != NULL) {
  //      lv_task_del(_serial_read_sensors_app_task);
   //     _serial_read_sensors_app_task = NULL;
   // }  

  //  sound_set_enabled_config( true );   //CURT ADD
 //   sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    
#ifdef NATIVE_64BIT

#else
  //  esp_wifi_set_promiscuous( false ); 
#endif
  //  wifictl_off();
    /**
     * restore display timeout time
     */
    display_set_timeout( wifimon_display_timeout );
}

static void wifimon_activate_cb( void ) {
    log_i("----------------- CURT -------- wifimon_activate_cb");

// ======================================= TURN SENSORS ON ====================================
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    delay(300);
 //   alarm_in_progress_start_alarm();

   // sound_set_enabled_config( true );
  //  sound_play_spiffs_mp3("/gui/sound/eyes.mp3");

  //  sound_set_enabled_config( true );   
  //  sound_play_progmem_wav(test_c_mouth_wav, test_c_mouth_wav_len);
    /**
     * restart wifi
     */
 //   wifictl_off();
    /**
     * setup promiscuous mode
     */

  //  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
 //   esp_wifi_init( &cfg );
 //   esp_wifi_set_country( &wifi_country );
//    esp_wifi_set_mode( WIFI_MODE_NULL ); 
 //   esp_wifi_start();
  //  esp_wifi_set_promiscuous( true );
  //  esp_wifi_set_promiscuous_rx_cb( &wifimon_sniffer_packet_handler );
    lv_roller_set_selected( channel_select, 0, LV_ANIM_OFF );
    wifimon_sniffer_set_channel( 1 );

    /**
     * start stats fetch task
     */
    //_wifimon_app_task = lv_task_create( wifimon_app_task, 100, LV_TASK_PRIO_MID, NULL );
    _wifimon_app_task = lv_task_create( wifimon_app_task, 100, LV_TASK_PRIO_MID, NULL );
   // _serial_read_sensors_app_task = lv_task_create( serial_read_sensors_app_task, 123, LV_TASK_PRIO_MID, NULL );   //=========== ADD TASK FOR SENSOR READ
    /**
     * save display timeout time
     */
    wifimon_display_timeout = display_get_timeout();
    display_set_timeout( DISPLAY_MAX_TIMEOUT );
}


/*******************************************************************************************************
********************************************************************************************************
********************************************************************************************************
                              ********* READ SENSOR DATA FROM SERIAL **********
                              ********* READ SENSOR DATA FROM SERIAL **********
                              ********* READ SENSOR DATA FROM SERIAL **********
                              ********* READ SENSOR DATA FROM SERIAL **********
                              ********* READ SENSOR DATA FROM SERIAL **********
********************************************************************************************************
********************************************************************************************************
*******************************************************************************************************/
/*
static void serial_read_sensors_app_task( lv_task_t * task ) {
    String content1 = "";
    char character1;
        
    delay(1);
    while(Serial1.available()) {
         character1 = Serial1.read();
         content1.concat(character1);
    }
    delay(25);
          
    String content2 = "";
    char character2;
        
    while(Serial2.available()) {
         character2 = Serial2.read();
         content2.concat(character2);
    }
    delay(25);
          
    //transfer new data to global var
    for(int i =0; i < content1.length(); i++){
        recievedSerial1[i] = content1[i];
    }

    for(int j =0; j < content2.length(); j++){
        recievedSerial2[j] = content2[j];
    }

}
*/
/*******************************************************************************************************
********************************************************************************************************
********************************************************************************************************
                              ********* MAIN LOOP **********
                              ********* MAIN LOOP **********
                              ********* MAIN LOOP **********
                              ********* MAIN LOOP **********
                              ********* MAIN LOOP **********
********************************************************************************************************
********************************************************************************************************
*******************************************************************************************************/
static void wifimon_app_task( lv_task_t * task ) {

    log_i("==== wifimon_app_task");
      
    log_i("RAW S1: %s\r", recievedSerial1);

    char bufferSerial1[numChars];
    int indexS1 = 0;
    bool dataStart = false;
    bool dataEnd = false;
   // char characterS1;

    // ============== EXTRACT MESSAGE =====================
    for(int a=0; a < numCharsInt; a++){
        char characterS1 = recievedSerial1[a];
      //  log_i("R: %c\n", characterS1);
        if(characterS1 == '<' && dataStart == false){             //beggining of message?
             dataStart = true;             //record start message
        } else if( dataStart == true && dataEnd == false ){    //if message already started proceed
            if(characterS1 == '>'){        //message over?
                dataEnd = true;             //record message over
                bufferSerial1[indexS1] = '\0';
            } else if(dataEnd == false){  //if not at end of message, proceed
                bufferSerial1[indexS1] = characterS1; //record message to buffer
              //  log_i("P: %c\n", bufferSerial1[indexS1]);
                indexS1++;
            }
        }
    }
    log_i("PARSED S1: %s\r", bufferSerial1);

//new way 
    /*
    int varCount = 0;
    int individualIndex = 0;
    char var1[] = "0";
    char var2[] = "0";
    char var3[] = "0";
    if(dataEnd == true){
        for(int p=0; p<indexS1; p++){
            if(bufferSerial1[p] == ','){

                if(varCount == 0){ //terminate string
                    var1[individualIndex] = '\0';
                } else if(varCount == 1){
                    var2[individualIndex]= '\0';
                } else if(varCount == 2){
                    var3[individualIndex]= '\0';
                }

              varCount++; //next var
              individualIndex = 0; //reset
            } else if(varCount == 0){
                var1[individualIndex] = bufferSerial1[p];
                individualIndex++;
            } else if(varCount == 1){
                var2[individualIndex] = bufferSerial1[p];
                individualIndex++;
            } else if(varCount == 2){
                var3[individualIndex] = bufferSerial1[p];
                individualIndex++;
            }
        }
        log_i("sep strings MLX15: %s %s %s", var1, var2, var3);
    }
*/

//old way
    if(dataEnd == true){

        // =============== EXTRACT VARIABLES ===================
        char * strtokIndx; // this is used by strtok() as an index
        strtokIndx = strtok(bufferSerial1,",");   
        dataMLX15[0] = atof(strtokIndx);     // convert this part to a float

        strtokIndx = strtok(NULL, ",");
        dataMLX15[1] = atof(strtokIndx);     // convert this part to a float

        strtokIndx = strtok(NULL, ",");
        dataMLX15[2] = atof(strtokIndx);     // convert this part to a float

        log_i("dataMLX15: %f %f %f", dataMLX15[0], dataMLX15[1], dataMLX15[2]);

        data = (int)dataMLX15[0];
        mgmt = (int)dataMLX15[1];
        misc = (int)dataMLX15[2];

//map(value, fromLow, fromHigh, toLow, toHigh)
        data = map(data, 66, 102, 0, 100);
        mgmt = map(mgmt, 66, 102, 0, 100);
        misc = map(misc, 66, 102, 0, 100);
/*
        int max = data;
        int min = data;

        if(mgmt < min){ min = mgmt;}
        if(misc < min){ min = misc;}

        if(mgmt > max){ max = mgmt;}
        if(misc > max){ max = misc;}

        data = (data - min + 5)*3;
        mgmt = (mgmt - min + 5)*3;
        misc = (misc - min + 5)*3;
        */
    }
      


//    log_i("RAW S2: %s\r", recievedSerial2);
/*
    char bufferSerial2[numChars];
    int indexS2 = 0;
    dataStart = false;
    dataEnd = false;
    char characterS2;

    for(int b=0; b < numCharsInt; b++){
        characterS2 = recievedSerial2[b];
        if(characterS2 == '<' && dataStart == false){             //beggining of message?
             dataStart = true;             //record start message
        } else if( dataStart == true && dataEnd == false ){    //if message already started proceed
            if(characterS2 == '>'){        //message over?
                dataEnd = true;             //record message over
            } else if(dataEnd == false){  //if not at end of message, proceed
                bufferSerial2[indexS2] = characterS2; //record message to buffer
                indexS2++;
            }
        }
    }

    log_i("PARSED S2: %s\r", bufferSerial2);

*/
    delay(2);






//Serial1.write("111");
//Serial2.write("222");
//delay(100);

    String content1 = "";
    char character1;
        
    delay(1);
    while(Serial1.available()) {
         character1 = Serial1.read();
         content1.concat(character1);
    }
    delay(25);
          /*
    String content2 = "";
    char character2;
        
    while(Serial2.available()) {
         character2 = Serial2.read();
         content2.concat(character2);
    }
    delay(25);
    */
          
    //transfer new data to global var
    for(int i =0; i < content1.length(); i++){
        recievedSerial1[i] = content1[i];
    }
/*
    for(int j =0; j < content2.length(); j++){
        recievedSerial2[j] = content2[j];
    }
*/

    /********************************************************************************************************/
    /**************************************** READ BUILT IN IMU *********************************************/
    /********************************************************************************************************/
    /*
        TTGOClass * ttgo = TTGOClass::getWatch();

    Accel acc;
    ttgo->bma->getAccel(acc);
    log_i("acc.x: %d", acc.x);
    log_i("acc.y: %d", acc.y);
    */

    //int16_t x = acc.x * MOUSE_SENSIVITY;
    //int16_t y = acc.y * MOUSE_SENSIVITY;
    //mgmt = ((acc.x + 1000) / 20);
    //data = ((acc.y + 1000) / 20);

    if( mgmt < 0 ) mgmt = 0; 
    if( data < 0 ) data = 0; 
    if( misc < 0 ) misc = 0; 
    if( mgmt > 100 ) mgmt = 100; 
    if( data > 100 ) data = 100; 
    if( misc > 100 ) misc = 100; 

    //********************************************* CURT ADD AUDIO ***********************************************************************************************************
    if(mgmt > 85){
    //    wifimon_test_play_sound();
    }
    if(data > 85){
    //    wifimon_test_play_sound();
    }

    // ------------------ END CURT ADD ------------------


    /**
     * add seria data
     */
    lv_chart_set_next(chart, ser1, mgmt);
    lv_chart_set_next(chart, ser2, data);
    lv_chart_set_next(chart, ser3, misc);
    /**
     * refresh chart
     */
    lv_chart_refresh(chart);
    /**
     * reset packet counter
     */
    data = 0;
    mgmt = 0;
    misc = 0; 
}

