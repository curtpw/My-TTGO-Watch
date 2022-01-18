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
#include "hardware/powermgm.h" //CURT



lv_obj_t *example_app_main_tile = NULL;

lv_task_t * _example_app_task_5000;
lv_task_t * _example_app_task_10000;
lv_task_t * _example_app_play_sound_task = nullptr;

LV_IMG_DECLARE(refresh_32px);
LV_FONT_DECLARE(Ubuntu_72px);

static void exit_example_app_main_event_cb( lv_obj_t * obj, lv_event_t event );
static void enter_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event );
void example_app_task_5000( lv_task_t * task );
void example_app_task_10000( lv_task_t * task );

static void example_app_play_sound_task( lv_task_t * task );

void example_app_main_setup( uint32_t tile_num ) {

    example_app_main_tile = mainbar_get_tile_obj( tile_num );

    lv_obj_t * exit_btn = wf_add_exit_button( example_app_main_tile, exit_example_app_main_event_cb );
    lv_obj_align(exit_btn, example_app_main_tile, LV_ALIGN_IN_BOTTOM_LEFT, THEME_ICON_PADDING, -THEME_ICON_PADDING );

    lv_obj_t * setup_btn = wf_add_setup_button( example_app_main_tile, enter_example_app_setup_event_cb );
    lv_obj_align(setup_btn, example_app_main_tile, LV_ALIGN_IN_BOTTOM_RIGHT, -THEME_ICON_PADDING, -THEME_ICON_PADDING );

    // create an task that runs every 5 secounds
//    _example_app_task_5000 = lv_task_create( example_app_task_5000, 5000, LV_TASK_PRIO_MID, NULL );

    // create an task that runs every 10 secounds
    _example_app_task_10000 = lv_task_create( example_app_task_10000, 10000, LV_TASK_PRIO_MID, NULL );
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
    sound_play_progmem_wav( test_c_mouth_wav, test_c_mouth_wav_len ); 
    //sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    //motor_vibe(100); 
}

/*
void example_app_task_5000( lv_task_t * task ) {
    log_i("----------------- CURT -------- example_app_task_5000");
    // put your code her
    //turn sound off
    if( _example_app_play_sound_task!=nullptr) {
        lv_task_del( _example_app_play_sound_task );
        _example_app_play_sound_task = nullptr;
    }     
}
*/

void example_app_task_10000( lv_task_t * task ) {
    log_i("----------------- CURT -------- example_app_task_10000");
  //  _example_app_play_sound_task = lv_task_create( example_app_play_sound_task, 3000, LV_TASK_PRIO_MID, NULL ); //play sound
    // put your code her
}