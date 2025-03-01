/*
 *  keyb.c
 * 
 */

#define DEBUG

#include "config.h"

#include "keyb.h"

#include "md380.h"
#include "debug.h"
#include "netmon.h"
#include "mbox.h"
#include "console.h"
#include "syslog.h"
#include "lastheard.h"
#include "radio_config.h"
#include "sms.h"
#include "beep.h"
#include "codeplug.h"
#include "radiostate.h"
#include "printf.h"
#include "menu.h" // create_menu_entry_set_tg_screen() called from keyb.c !
#if( CONFIG_MORSE_OUTPUT ) 
# include "narrator.h"  // 'storyteller', triggerable via sidekey
#endif
#if( CONFIG_APP_MENU )
# include "app_menu.h" // optional 'application' menu, activated by red BACK-button
#endif
#include <stdint.h>
#include "amenu_set_tg.h"
#include "amenu_codeplug.h"
#include "spiflash.h"

extern int fIsEditing;

uint8_t kb_backlight=0; // flag to disable backlight via sidekey.
// Other keyboard-related variables belong to the original firmware,
// e.g. kb_keypressed, address defined in symbols_d13.020 (etc).


// Values for kp
// 1 = pressed
// 2 = release within timeout
// 1+2 = pressed during rx
// 4+1 = pressed timeout
// 8 = rearm
// 0 = none pressed

#if defined(FW_D13_020) || defined(FW_S13_020)
inline int get_main_mode()
{
    return gui_opmode1 & 0x7F ;
}

void reset_backlight()
{
    // struct @ 0x2001dadc
    backlight_timer = md380_radio_config.backlight_time * 500 ;

#if defined(FW_D13_020)
    // enabling backlight again.
    void (*f)(uint32_t,uint32_t) = (void*)( 0x0802b80a + 1 ); // S: ??? 0x0802BAE6
    f(0x40020800,0x40);
#elif defined(FW_S13_020)
    // enabling backlight again on MD390/G in monitor mode
    void (*f)(uint32_t,uint32_t) = (void*)( 0x0802bae6 + 1 ); // S: ??? 0x0802BAE6
    f(0x40020800,0x40);
    // # warning please consider hooking. // too many warnings - see issue #704 on github
#else //TODO add support for other firmware, e.g. D02.032 (?)

#endif
}

int beep_event_probe = 0 ;

void switch_to_screen( int scr )
{
    // cause transient -> switch back to idle screen.
    gui_opmode2 = OPM2_MENU ;
    gui_opmode1 = SCR_MODE_IDLE | 0x80 ;
    
    nm_screen = scr ;
}


void copy_dst_to_contact()
{ 
#if defined(FW_D13_020) || defined(FW_S13_020)
    int dst = rst_dst ;
    extern wchar_t channel_name[] ;
    
    contact.id_l = dst & 0xFF ;
    contact.id_m = (dst>>8) & 0xFF ;
    contact.id_h = (dst>>16) & 0xFF ;
    
    //wchar_t *p = (void*)0x2001e1f4 ;
    wchar_t *p = (void*)channel_name ;
    
    if( rst_grp ) {
        contact.type = CONTACT_GROUP ;        
        snprintfw( p, 16, "TG %d", dst );
    } else {
        snprintfw( p, 16, "U %d", dst );
        contact.type = CONTACT_USER ;        
    }

    /* I can't see how this doesn't crash, as draw_zone_channel() is
       an even address. --Travis
    */
    extern void draw_zone_channel(); // TODO.
    
    draw_zone_channel();

	ad_hoc_talkgroup = dst;
	ad_hoc_call_type = CONTACT_GROUP;
	ad_hoc_tg_channel = channel_num;
	CheckTalkgroupAfterChannelSwitch();
#else
#endif
}


//#if defined(FW_D13_020) || defined(FW_S13_020)
#if defined(FW_S13_020)
extern void gui_control( int r0 );
#else
#define gui_control(x)
#endif

void handle_hotkey( int keycode )
{
	//char lat[23] = { 0 };
	//char lng[23] = { 0 };
    reset_backlight();
	//char stufff = 0;
	channel_info_t *ci = &current_channel_info;
	//int ts2 = (ci->cc_slot_flags >> 3) & 0x1;
    
    switch( keycode ) {
		case 0 :
			clog_redraw();
            switch_to_screen(6);
            break ;
        case 1 :
			//syslog_printf("=key_code=%d =%d - %d\n", kb_keycode, kb_keypressed, kb_key_press_time);
			if (global_addl_config.audio_leveling) {
				syslog_printf("AudLvling=off\n");
				bp_send_beep(BEEP_ROGER);
			}
			else {
				syslog_printf("Audio_Test=on\n");
				bp_send_beep(BEEP_TEST_3);
			}
			global_addl_config.audio_leveling = (global_addl_config.audio_leveling ? 0 : 1);
            //sms_test();
			//c5000_spi0_writereg( 0x0F, 0xE8);

			/*current_channel_info.cc_slot_flags = (ts2 ? (current_channel_info.cc_slot_flags & 0x7) | 0x4 : (current_channel_info.cc_slot_flags & 0xB) | 0x8);

			
			

			c5000_spi0_readreg(0x10, &stufff);
			stufff = (stufff & 0xFE) | (ts2 ? 0x1 : 0);
			c5000_spi0_writereg(0x10, stufff);
			
			bp_send_beep(BEEP_TEST_1);*/

			//con_printf("%S\r\n", (wchar_t*)0x2001E1A0);
			/*int i = 0;

			

#if defined(FW_S13_020)

			//typedef void stuff(void);
			//stuff* f = (stuff*)0x8016850;
			//f();

			char *c = (char*)0x2001E1A0;
#else
			char *c = (char*)0x2001E1A0;
#endif
			for (; *c != 0 && *c != 0xFF; c += 2) {
				syslog_printf("%c", *c);
				lat[i] = *c;
			}
			c += 2;
			i = 0;
			syslog_printf("\n");
			for (; *c != 0 && *c != 0xFF; c += 2) {
				syslog_printf("%c", *c);
				lng[i] = *c;
			}
			syslog_printf("\n\n");*/

			/*char nVal = 0xC1, oVal=0;
			md380_spiflash_read(&oVal, 0x7987, 1);
			syslog_printf("Val: %02X\r\n", oVal);
			md380_spiflash_write(&nVal, 0x7987, 1);*/

            break ;
		case 2 :
			slog_redraw();
            switch_to_screen(5);
            break ;
        case 3 :
            copy_dst_to_contact();
            break ;
        case 4 :
            lastheard_redraw();
            switch_to_screen(4);
            break ;
        case 5 :
            syslog_clear();
			lastheard_clear();
			slog_clear();
			clog_clear();
			slog_redraw();
			nm_started = 0;	// reset nm_start flag used for some display handling
			nm_started5 = 0;	// reset nm_start flag used for some display handling
			nm_started6 = 0;	// reset nm_start flag used for some display handling
            break ;
        case 6 :
        {
            static int cnt = 0 ;
            syslog_printf("=dump %d=\n",cnt++);
        }
            syslog_dump_dmesg();
            break ;
		case 13 : //end call
            //bp_send_beep(BEEP_TEST_1);
			if(nm_screen){
				//channel_num = 0;
				switch_to_screen(0);
				//if(Menu_IsVisible()){
				//	channel_num = 0;
				//}
			}else if(!Menu_IsVisible()){
				switch_to_screen(9);
				switch_to_screen(0);
			}
            break ;
		case 7 :
			//Let 7 disable ad-hoc tg mode;
			if (!nm_screen && !Menu_IsVisible()) {
				ad_hoc_tg_channel = 0;
			}
			if(nm_screen){
				//bp_send_beep(BEEP_TEST_1);
				switch_to_screen(0);
				//channel_num = 0;
			}else if(!Menu_IsVisible()){
				switch_to_screen(9);
				switch_to_screen(0);
			}
			
			break;
        case 8 :
            //bp_send_beep(BEEP_TEST_2);
            switch_to_screen(1);
            break ;
        case 9 :
            //bp_send_beep(BEEP_TEST_3);
            switch_to_screen(2);
            break ;
        case 11 :
            //gui_control(1);
            //bp_send_beep(BEEP_9);
            //beep_event_probe++ ;
            //sms_test2(beep_event_probe);
            //mb_send_beep(beep_event_probe);
            break ;
        case 12 :
            //gui_control(241);
            //bp_send_beep(BEEP_25);
            //beep_event_probe-- ;
            //sms_test2(beep_event_probe);
            //mb_send_beep(beep_event_probe);
            break ;
		case 14 :
			if (*(char*)0x2001E94F > 3) {
				md380_menu_0x2001d3ee = 0; //  startpos cursor
				md380_menu_0x2001d3ef = 0; //  startpos cursor
				uint8_t *p;
				for (int i = 0; i < 0x11; i++) {
					p = (uint8_t *)md380_menu_edit_buf;
					p = p + i;
					*p = 0;
				}
				break;
			}

			switch_to_screen(9);
			//channel_num=0;
			draw_rx_screen(0xff8032);
			break;
			
			// key '#'
        case 15 :
			if( !Menu_IsVisible() && nm_screen){
				syslog_redraw();
				switch_to_screen(3);
			}
            break ;
    }    
}

void handle_sidekey( int keycode, int keypressed )
{
    if ( keycode == 18 ) {             //top button
      if ( (keypressed & 2) == 2 && kb_top_side_key_press_time < kb_side_key_max_time) {  //short press
        evaluate_sidekey( top_side_button_pressed_function );
      }
      else if ( keypressed == 5) {     //long press
        evaluate_sidekey( top_side_button_held_function );
      }
    }
    else if ( keycode == 17 ) {        //bottom button
      if ( (keypressed & 2) == 2 && kb_bot_side_key_press_time < kb_side_key_max_time) { //short press
      evaluate_sidekey( bottom_side_button_pressed_function );
    }
    else if ( keypressed == 5 ) {      //long press
      evaluate_sidekey( bottom_side_button_held_function );
    }
    }
}

void evaluate_sidekey( int button_function) // This is where new functions for side buttons can be added
{
	int zoneIndex = 0;
  switch ( button_function ) {  // We will start at 0x50 to avoid conflicting with any added functions by Tytera.
    case 0x50 :                 // Toggle backlight enable pin to input/output. Disables backlight completely.
      #if (CONFIG_DIMMED_LIGHT) // If backlight dimmer is enabled, we will use that instead.
        kb_backlight ^= 0x01;   // flag for SysTick_Handler() to turn backlight off completely
      #else
        GPIOC->MODER = GPIOC->MODER ^ (((uint32_t)0x01) << 12);
      #endif
      reset_backlight();
      break;
    case 0x51 :    // "Set Talkgroup"
      //create_menu_entry_set_tg_screen(); 
      // Creating the menu entry seems ok here, 
      // but it's impossible (or at least unsafe) to ENTER / invoke the menu from here.
      // Call stack: kb_handler_hook() -> handle_sidekey() -> evaluate_sidekey()
      //               |__ in D13.020, patched to address 0x0804ebd2, thus
      //                   called from task 'biglist_pollsubsys_maybe', when the
      //                   shared keyboard/LCD interface is configured to poll the keyboard,
      //                   not to 'drive the display'. See the monster-disassembly.
#    if (CONFIG_APP_MENU)
	  Menu_Open(NULL/*default instance*/, NULL/*main items*/, "TkGrp"/*cpJumpToItem*/, APPMENU_EDIT_OVERWRT);
	  backlight_timer = md380_radio_config.backlight_time * 500;
#    endif
      break;
#  if( CONFIG_MORSE_OUTPUT )    // optional feature - see config.h 
    case 0x52 : // starts the 'Morse narrator' via programmable button ("on request")
      narrator_start_talking(); // doesn't call anything in the original firmware
      break;
    case 0x53 : // repeats the last 'Morse anouncement' (short, not the full story)
      narrator_repeat();
      break;
#  endif

    case 0x54 : // toggle promiscuous mode
      global_addl_config.promtg = (global_addl_config.promtg==0? 1: 0);
      cfg_save();
      break;

	case 0x55: // adhoc private call
		ad_hoc_talkgroup = rst_src;
		ad_hoc_tg_channel = channel_num;
		ad_hoc_call_type = CONTACT_USER;
		CheckTalkgroupAfterChannelSwitch();
		break;

	case 0x56: // zone inc
		zoneIndex = ZoneList_GetCurrentIndex();
		if (!ZoneList_SetZoneByIndex(zoneIndex + 1)) {
			for (int i = 0; i < CODEPLUG_MAX_ZONE_LIST_ENTRIES; i++) {
				if (ZoneList_SetZoneByIndex(i)) {
					break;
				}
			}
		}
		break;

	case 0x57: // zone dec
		zoneIndex = ZoneList_GetCurrentIndex();
		if (zoneIndex > 0 && ZoneList_SetZoneByIndex(zoneIndex - 1)) {
			break;
		}
		else {
			for (int i = CODEPLUG_MAX_ZONE_LIST_ENTRIES-1; i >= 0; i--) {
				if (ZoneList_SetZoneByIndex(i)) {
					break;
				}
			}
		}
		break;
	case 0x58:    // "Open Lastheard"
#    if (CONFIG_APP_MENU)
		Menu_Open(NULL/*default instance*/, NULL/*main items*/, "Lastheard"/*cpJumpToItem*/, APPMENU_EDIT_OFF);
		backlight_timer = md380_radio_config.backlight_time * 500;
#    endif
		break;

    default:
      return;
  }

  kb_keypressed = 8 ; // Sets the key as handled. The firmware will ignore this button press now.
}

void trace_keyb(int sw)
{
    static uint8_t old_kp = -1 ;
    uint8_t kp = kb_keypressed ;
    
    if( old_kp != kp ) {
        //LOGB("kp: %d %02x -> %02x (%04x) (%d)\n", sw, old_kp, kp, kb_row_col_pressed, kb_keycode );
        old_kp = kp ;
    }
}



inline int is_intercept_allowed()
{
	if ((*(char*)0x2001E94F > 3) && kb_keycode == 14) {
		return 1;
	}
    if( !is_netmon_enabled() || Menu_IsVisible()) {
        return 0 ;
    }
    
//    switch( get_main_mode() ) {
//        case 28 :
//            return 1 ;
//        default:
//            return 0 ;
//    }
    
	//2001CB98 = D13 - edit buffer

    switch( gui_opmode2 ) {
        case OPM2_MENU :
            return 0 ;
        //case 2 : // voice
        //case 4 : // terminate voice
        //    return 1 ;
        default:
            return 1 ;
    }
}

inline int is_intercepted_keycode( int kc )
{
    switch( kc ) {
		case 0 :
        case 1 :
		case 2 :
        case 3 :
        case 4 :
        case 5 :
        case 6 :
        case 7 :
        case 8 :
        case 9 :
        case 11 :
        case 12 :
		case 13 : //end call
		case 14 : // *
        case 15 :
            return 1 ;
        default:
            return 0 ;
    }    
}

inline int is_intercepted_keycode2(int kc)
{
	switch (kc) {
	
	case 20:
	case 21:
	case 13: //end call
		return 1;
	default:
		return 0;
	}
}
#endif

extern void kb_handler();

static int nextKey = -1;

void kb_handle(int key) {
	//int kp = kb_keypressed;
	int kc = key;

	if (is_intercept_allowed()) {
		if (is_intercepted_keycode2(kc)) {
			//if ((kp & 2) == 2) {
				//kb_keypressed = 8;
				handle_hotkey(kc);
				return;
			//}
		}
	}

	if (key == 11 || key == 12) {
		kb_keycode = key;
		kb_keypressed = 2;
	}

}

int lastKey = 0;
void kb_handler_hook()
{

#if defined(FW_D13_020) || defined(FW_S13_020)
    trace_keyb(0);

    kb_handler();

    trace_keyb(1);

	//Menu_OnKey(KeyRowColToASCII(kb_row_col_pressed));

	

	if (nextKey > 0) {
		kb_keypressed = 2;
		kb_keycode = nextKey;
		nextKey = -1;
	}
    
    int kp = kb_keypressed ;
    int kc = kb_keycode ;

	if (kc == 20 || kc == 21) {
		kb_keypressed = 8;
		return;
	}

    // allow calling of menu during qso.
    // not working correctly.
    if( global_addl_config.experimental ) {
        if( (kp & 2) == 2 ) {
            if( gui_opmode2 != OPM2_MENU ) {
                gui_opmode2 = OPM2_MENU ;
                gui_opmode1 = SCR_MODE_IDLE | 0x80 ;
            }
        }
    }

	
		if (is_intercept_allowed()) {
			if (is_intercepted_keycode(kc)) {
				if ((kp & 2) == 2) {
					if (kc == 1 && lastKey == kc) {
						kb_keypressed = 8;
						return;
					}else {
						kb_keypressed = 8;
						
						handle_hotkey(kc);
						lastKey = kc;
						return;
					}
				}
			}
		}
		if ((kp & 2) != 2) {
			lastKey = 0;
		}

    if ( kc == 17 || kc == 18 ) {
      if ( (kp & 2) == 2 || kp == 5 ) { // The reason for the bitwise AND is that kp can be 2 or 3
        handle_sidekey(kc, kp);         // A 2 means it was pressed while radio is idle, 3 means the radio was receiving
        return;
      }
    }

#  if( CONFIG_APP_MENU )  // prevent opening "green key menu" as long as our "alternative menu" is open

	

    if( Menu_IsVisible() )  
     { 
       if( (kp & 2) == 2 ) 
        { kb_keypressed = 8; // Sets the key as handled. The firmware will ignore this button press now.
          // (would be nice if it did. But Tytera's original menu still flashed up
          //  when leaving our ALTERNATIVE menu (aka 'app menu') via GREEN key)
        }
       return; // "return early", don't let kb_handler() process 'kb_row_col_pressed' at all .
     }
#  endif // CONFIG_APP_MENU ?


#else //TODO add support for other firmware, e.g. D02.032 (?)
    // # warning please consider hooking. // too many warnings - see issue #704 on github
    return;
#endif
}
