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
#include "example_app_setup.h"
#include "gui/mainbar/mainbar.h"
#include "gui/statusbar.h"
#include "gui/widget_factory.h"
#include "gui/widget_styles.h"

#include <Arduino.h>


lv_obj_t *example_app_setup_tile = NULL;
lv_obj_t *example_app_foobar_switch = NULL;

static void exit_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event );
static void example_app_foobar_switch_event_cb( lv_obj_t * obj, lv_event_t event );

void example_app_setup_setup( uint32_t tile_num ) {
    log_i("----------------- CURT -------- example_app_setup_setup");
    /**
     * get tile obj from tile number
     */
    example_app_setup_tile = mainbar_get_tile_obj( tile_num );
    /**
     * add setup header with exit button
     */
    lv_obj_t *header = wf_add_settings_header( example_app_setup_tile, "my app setup", exit_example_app_setup_event_cb );
    lv_obj_align( header, example_app_setup_tile, LV_ALIGN_IN_TOP_LEFT, THEME_PADDING, THEME_PADDING );
    /**
     * setup test switch
     */
    lv_obj_t *example_app_foobar_switch_cont = wf_add_labeled_switch( example_app_setup_tile, "my app test switch", &example_app_foobar_switch, false, example_app_foobar_switch_event_cb, SETUP_STYLE );
    lv_obj_align( example_app_foobar_switch_cont, header, LV_ALIGN_OUT_BOTTOM_MID, 0, THEME_PADDING );
}

static void example_app_foobar_switch_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_VALUE_CHANGED ): EXAMPLE_APP_INFO_LOG( "switch value = %d\r\n", lv_switch_get_state( obj ) );
                                        log_i("----------------- CURT -------- example_app_foobar_switch_event_cb");
                                        break;
    }
}

static void exit_example_app_setup_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_CLICKED ):       mainbar_jump_back();
                                        log_i("----------------- CURT -------- exit_example_app_setup_event_cb");
                                        break;
    }
}
