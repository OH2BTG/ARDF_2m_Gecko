/* SPDX-License-Identifier: MIT */
// ARDF Gecko v012
// OH2BTG versions. tehty volumesäätö joka tottelee dBm arvoa
// E10 kokeilua isommilla fonteilla
// FreeRTOS
// 23112024 ARDF v16
// 25112024 ARDF v17 display palkki ja att/setup kytkin toimii
// 14122024 ARDF v20 Attn 35dB / 65dB Working
// 31122024 ARDF v24
// 05012025 ARDF v25 Changed PA2 input to F11. PA2 is for battery meter
// 08012025 ARDF v26 Battery reading works
// 15012025 ARDF v27 volume_normal 1 mean hand adjust added
// 19012025 ARDF v28 RSSI color bar added
// 28012025 ARDF V30 Added set1 for user config info for saving flash
// 08022025 ARDF V31 Waterfall changed to show p.volume history
// 04032025 ARDF V32 AGC is adjusted by RSSI 
// 06042025 ARDF V33 Added ext att back PB12 is 65dB attn and PA1 is 25dB attn
// 13042025 ARDF V34 First time used in Tampere 2m Pyynikki and foxes find OK
// 01052025 ARDF V34 PA0 and PA1 ports changed from UART use, see USART0_enter_DefaultMode_from_RESET
// 02052025 ARDF V35 Backcolour changed to white
// 22062025 ARDF V36 Waterfall display taken in use 
// 25062025 ARDF V36 Waterfall / S-meter switch added to menu 
// 08072025 SRDF V37 Added ARDF mode ON / OFF. It changes AGC function  
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "display.h"
#include "rig.h"
#include "ui.h"
#include "ui_hw.h"
#include "ui_parameters.h"
#include "dsp.h"
#include "dsp_driver.h" // OH2BTG 
#include "power.h"
#include "railtask.h"
#include "config.h"
#include "font8x8_basic.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_msc.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define USERDATA ((uint32_t*)USERDATA_BASE)
#define USERDATA_SIZE 2048

rig_parameters_t p = {
	.keyed = 0,
	.mode = MODE_AM,
	.frequency = RIG_DEFAULT_FREQUENCY,
	.split_freq = 0000000,
	.offset_freq = 0,
	.volume = 0,
	.waterfall_averages = 20,
	.squelch = 20, // auki
	.ctcss = 118,
};
rig_status_t rs = {0};

// CTCSS frequencies in multiples of 0.1 Hz
// List from https://en.wikipedia.org/wiki/Continuous_Tone-Coded_Squelch_System#List_of_tones
const uint16_t ctcss_freqs[] = {
	0, 670, 693, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273, 1318, 1365, 1413, 1462, 1500, 1514, 1567, 1598, 1622, 1655, 1679, 1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995, 2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541
};

int mem_prev = RIG_DEFAULT_FREQUENCY; // ARDF memory 0 saved
bool att_25dB_status = false;
bool att_35dB_status = false;
bool att_65dB_status = false;
float RSSI_mov_average = 0.0;
float logValue = 1;
int ARDF_disp_on = 0;
int attn_step = 1;
int ARDF_scale_fox_pwr = 0;
uint32_t set1 = 0xffffffff;// Max value. 0xffffffff ARDF set1 is for store user configuration values to flash memory
uint32_t set1read = 0;
int RSSI_bar_R = 0;
int RSSI_bar_G = 0;
int RSSI_bar_B = 0;
int offset_cursor_ready = 0; // ARDF ready to draw 
uint32_t BattIntegerPart = 0;    // Integer part of battery voltage
uint32_t BattFractionalPart = 0; // Fractional part (2 decimal places)
int set1_last_bit = 0;
int volume = 20;
int agc_vol_max = 1;
int agc_decay_l = 1;
bool set_ARDF_ON = false;
enum ui_field_name {
	// Common fields

	//UI_FIELD_FREQ0,
	UI_FIELD_FREQ1,
	UI_FIELD_FREQ2,
	UI_FIELD_FREQ3,
	UI_FIELD_FREQ4,
	UI_FIELD_FREQ5,
	UI_FIELD_FREQ6,
	UI_FIELD_FREQ7,
	UI_FIELD_FREQ8,
	UI_FIELD_FREQ9,
	UI_FIELD_MODE,
	UI_FIELD_WF_S_METER,
	UI_FIELD_MEM,
	UI_FIELD_MTR,
	UI_FIELD_VOL,	
	//UI_FIELD_VOL,
	
	
	//UI_FIELD_PTT,
	//UI_FIELD_VOL,
	//UI_FIELD_WF, // Waterfall averages
	//UI_FIELD_SPLIT0,
	//UI_FIELD_SPLIT1,
	

	// FM specific fields

	UI_FIELD_SQ, // Squelch
	UI_FIELD_CTCSS,

	// SSB specific fields

	// SSB finetune
	UI_FIELD_FT0,
	UI_FIELD_FT1,
	UI_FIELD_FT2,
	UI_FIELD_FT3,
};

// ARDF for vol display value store
typedef struct {
    int x, y, width, height;
    uint8_t r, g, b;
} ColorBlock;

#define MAX_COLOR_BLOCKS 2  // ARDF Store both the black and colored bar

ColorBlock color_blocks[MAX_COLOR_BLOCKS];
int color_block_count = 0;

struct ui_field {
	enum ui_field_name name;
	unsigned char pos1, pos2, color;
	const char *tip;
};

struct ui_view {
	// Number of fields
	size_t n;
	// Function to format text for the view
	int (*text)(char*, size_t);
	// Array of fields
	struct ui_field fields[];
};

#define TEXT_LEN 64 // kokeilua OH2BTG oli 64

struct ui_state {
	// Current UI view
	const struct ui_view *view;
	// Previous encoder position
	unsigned pos_prev;
	//unsigned pos_diff;
	int backlight_timer;
	// Number of currently selected field
	unsigned char cursor;
	unsigned char keyed;
	unsigned char button_prev, ptt_prev, keyed_prev;
	unsigned char ardf_F11_prev, ardf_pa5_prev; // ARDF OH2BTG
	bool ardf_pa5_setup;
	// Index to ctcss_freqs
	unsigned char ctcss;
	char text[TEXT_LEN+1];
	unsigned char color[TEXT_LEN+1];
	char textprev[TEXT_LEN+1];
	unsigned char colorprev[TEXT_LEN+1];
};

struct ui_state ui;

// UI fields common to every mode. ARDF set cursor to right place for selection
// To simplify code, these are repeated in every ui_view struct. x-alku,x-loppu,digits
// ARDF removed GHz and last 3 digits from freq. Row define what is menu order in display 
#define UI_FIELDS_COMMON \
	{ UI_FIELD_FREQ1,     0, 0, 2, "Freq 100 MHz"     },\
	{ UI_FIELD_FREQ2,     1, 1, 2, "Freq 10 MHz"      },\
	{ UI_FIELD_FREQ3,     2, 2, 2, "Freq MHz"         },\
	{ UI_FIELD_FREQ4,     3, 3, 2, "Freq 100 kHz"     },\
	{ UI_FIELD_FREQ5,     4, 4, 2, "Freq 10 kHz"      },\
	{ UI_FIELD_FREQ6,     5, 5, 2, "Freq kHz"         },\
	{ UI_FIELD_FREQ7,	  6, 6, 2, "Freq 100Hz"		  },\
	{ UI_FIELD_MEM,   	  8, 9, 2, "ARDF MEM"  		  },\
	{ UI_FIELD_MTR,	  	  24,27,4, "FOX PWR"	      },\
	{ UI_FIELD_MODE,      29,31,3, "Mode " 	   		  },\
	{ UI_FIELD_WF_S_METER,32,35,4, "Wfall Smeter"     }	
//	{ UI_FIELD_MTR,	  	  35,39,5, "FOX PWR"	      }	
//	{ UI_FIELD_WF_S_METER, 24,27,4, "Wfall Smeter"	  }	
//	{ UI_FIELD_MEM,   	  6, 7, 3, "ARDF OH2BTG"  	  }
//{ UI_FIELD_VOL,		 0, 0, 0,  "Volume "		  }
//	{ UI_FIELD_VOL,      24,27, 4, "Volume"           }
//	{ UI_FIELD_WF,       34,35, 2, "Waterfall"        }
#define UI_FIELDS_COMMON_N 13 // was 13
// sain lisättyä allaolevaan kentän TST OH2BTG  
static const char *const p_mode_names[] = {
	"MEM", " FM", " AM", "USB", "LSB", "---", "off","SET" // OH2BTG ver ja TST mode
};

static const char *const ARDF_memory[] = {
	"BE", "F1", "F2", "F3", "F4"
};


// Function to fetch a memory element by index
const char *call_ardf_memory(int index) {
	const char *result = call_ardf_memory(index);
    int size = sizeof(ARDF_memory) / sizeof(ARDF_memory[0]);
    if (index >= 0 && index < size) {
        
		//printf("ARDF Memory[%d]: %s\n",4, result); // Output: ARDF Memory[2]: 
		printf("ARDF_memory: %s\n", ARDF_memory[index]);
		return ARDF_memory[index];
    }
    return NULL; // Return NULL for an invalid index
}
int Wf_Smeter = 0; // ARDF set Waterwall as default display mode

static const char *const Wfall_S_Meter[] = {
    "WFal", "SMtr","Set ARDF","ARDF ON","ARDF OFF"
};

// Function to fetch menu element by index
const char *call_Wfall_SMeter(int index) {
    int size = sizeof(Wfall_S_Meter) / sizeof(Wfall_S_Meter[0]);
    if (index >= 0 && index < size) {
       // printf("Waterfall / S meter: %s\n", Wfall_S_Meter[index]); 
        return Wfall_S_Meter[index];
    }
    return NULL; // for invalid index 
}

int ardf_ON_OFF = 1; // ARDF mode default ON

static const char *const  Menu_ON_OFF[] = {  // ARDF MENU ON / OFF display
	"OFF","ON"
}; 

const char *call_Menu_ON_OFF(int index){  // ARDF MENU ON / OFF display
	int size = sizeof(Menu_ON_OFF)/ sizeof(Menu_ON_OFF[0]);
	if (index >=0 && index < size){
		return Menu_ON_OFF[index];
	}
}

// Function to convert dBm to meters based on a range
float true_dBm_max = -500;
float dBm_to_meters(float true_dBm_max, float in_min, float in_max, float out_min, float out_max) {
    return (true_dBm_max- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ARDF lookup table for convert dBm to distance 25 values
static const char *const ARDF_distance[] = {
"  5m","  7m"," 10m"," 20m"," 30m"," 50m"," 70m","100m",
"150m","200m","300m","500m","700m"," 1km",
" 1k5"," 2km"," 3km"," 4km"," 5km"," 6km"," 7km"," 8km"," 9km","10km","11km"
};


// Function to convert true_dBm_max to distance 

const char* convert_dBm_to_distance(float true_dBm_max) {
    // Using a dBm range of -120 to 0 to calculate index for the lookup table
	static int previous_index = -1;
	int true_dBm_max_4 = ((true_dBm_max - 60)/4) ;  // ARDF corrected -60 place where finetune dBm to meters
    int index = (int)dBm_to_meters(true_dBm_max_4, -120, 0,  sizeof(ARDF_distance) / sizeof(ARDF_distance[0]) - 1, 0); // ARDF kaava toimii
    // Ensure index is within bounds of the ARDF_distance array
    if (index < 0) index = 0;
    if (index >= sizeof(ARDF_distance) / sizeof(ARDF_distance[0])) index = sizeof(ARDF_distance) / sizeof(ARDF_distance[0]) - 1;
   // printf("ARDF true dBm max: %d\n", true_dBm_max_4);
  //  printf("ARDF index: %d\n", index); 	
    // Check if the index has changed
    if (index != previous_index) {
		p.volume = 0;
        audio_tone(1000,50);  // First numeber duration second frequency Play the audio tone
		//int global_amp_d = global_amp;
		//printf("global_amp: %5d \n",global_amp_d);
		//global_agc_amp -= 1000;
		//delay_ms(10);
		//audio_tone(500,100); 
		printf("ARDF_distance_change: %s\n", ARDF_distance[index]);
    previous_index = index;  // Update the previous index
    }
		index = index + ARDF_scale_fox_pwr;// scale 0.3W as reference 11 
		if(index <= 0)index = 0;
		if(index >=24)index = 24;
	//printf("ARDF_distance_adj: %d\n",index);	
    return ARDF_distance[index];

}

// ARDF lookup table for calibrating fox Pout to meters
static const char *const ARDF_Pout_mtr[] ={
"1uW","3uW","10uW","30uW","0.1mW","0.3mW","1mW","3mW","10mW","30mW","100mW","0.3W","1W","3W","10W","30W" // ARDF 0.3W on ref taso
};

// Function to calibrate fox tx pwr to distance

int ADJ_ARDF_fox_pwr = 0;
const char* cal_fox_pwr_to_mtr() {
	static int previous_index = -1;
	int index = ADJ_ARDF_fox_pwr;
    // Ensure index is within bounds of the ARDF_distance array
    //if (index < 0) index = 0;
    if (index >= sizeof(ARDF_Pout_mtr) / sizeof(ARDF_Pout_mtr[0])) index = sizeof(ARDF_Pout_mtr) / sizeof(ARDF_Pout_mtr[0]) - 1; 	
    // Check if the index has changed
	 const int max_index = sizeof(ARDF_Pout_mtr) / sizeof(ARDF_Pout_mtr[0]) - 1;	
		if (index < 0) {
        index = 0;
    	}
    if (index > max_index) {
        index = max_index;
    	}
    if (index != previous_index) {
	//	printf("ARDF_Pout_mtr_index: %d\n", index);
	//	printf("ARDF_Pout_mtr: %s\n", ARDF_Pout_mtr[index]);

    previous_index = index;  // Update the previous index
    }
	//if(index <= 0)index = 0;
	//if(index >= 15)index = 15;
	set1 = index;
    return ARDF_Pout_mtr[index];
}


//static const char *const p_keyed_text[] = { "rx", "tx" };

float max_dBm = -520; // OH2BTG ARDF set minimum dBm for comparison
int true_dBm = -520;
int true_dBm2 = -520;
int true_dBm3 = -520;
int true_dBm_max2 = -500;
int true_dBm_max3 = -500;
int attn_state = 1;
int memARDF = 0; // ARDF niin monta kuin on muistipaikkaa
//int mem5;
int Volume_normal = 0; // ARDF vol adjust by hand = 1 by signal = 0;
uint32_t num_values = 10; // ARDF montako muistipaikkaa halutaan käyttää
bool flash_read_done = false;
uint8_t vol_line[128 * 3];
int agc_amp_offset = 1;  

// Format text common for all views
static int ui_view_common_text(char *text, size_t maxlen){ // ARDF Teksti näyttö
	//int global_amp_d = global_amp;
	//printf("extern float global amp %d \n",global_amp_d);
	unsigned freq = p.frequency;
	int keyed = p.keyed;
	enum rig_mode mode = p.mode;
	ARDF_RSSI(); // ARDF fetching RSSI from railtask
	int dot_dBm = RSSI_read_desimal();
	//printf("RSSI RAW: %d \n", RSSI_read);
	//printf("RSSI: %d,%d dBm \n", (RSSI_read / 4) - 3, dot_dBm);  // ARDF säädetään dBm näyttö oikein
    adjust_attn(); // ARDF test
	const char *ARDF_txpwr_to_mtr = cal_fox_pwr_to_mtr();
	const char *ARDF_meters = (convert_dBm_to_distance(true_dBm_max));
	const char *Wfall_S_Meter = call_Wfall_SMeter(Wf_Smeter);
	const char *ARDF_ON_OFF = call_Menu_ON_OFF(ardf_ON_OFF);
	//printf("ARDF_meters %s\n",ARDF_meters);
	//char ARDF_meters_d = ARDF_meters + ARDF_scale_fox_pwr; 
	//if (volume < 1 ) volume = 1;	
	if (true_dBm_max <= -520)true_dBm_max = -510;
	dsp_update_params();
		if (keyed)
		freq += p.split_freq;
	if (mode >= MODE_USB && mode <= MODE_LSB) // ARDF MODE_CWL changed to LSB
		freq += p.offset_freq;
	int true_dBm_max_d = true_dBm_max; // important to change float to int for display.
	int global_agc_amp_d = global_agc_amp; // important to change float to int for display.
	//int global_amp_d = global_amp;
	unsigned freq_rounded_100Hz = freq / 1000;  // ARDF last 3 digits out
	return snprintf(text, maxlen,		
		 "%6u  %2s%4s  %4d%4d%5s%2s%4s    ",  //40 characters
		//"1234567890123456789012345678901234567890"
		freq_rounded_100Hz,ARDF_memory[memARDF],ARDF_meters, true_dBm / 4, (true_dBm_max_d /4 ),ARDF_txpwr_to_mtr, p_mode_names[mode]
		,Wfall_S_Meter // ARDF volume removed
		);
} 

static int ui_view_ARDF_text(char *text, size_t maxlen){ // ARDF Teksti näyttö
	unsigned freq = p.frequency;
	int keyed = p.keyed;
	enum rig_mode mode = p.mode;
	ARDF_RSSI(); // ARDF fetching RSSI from railtask
	BattV_read();
    adjust_attn(); // ARDF test
	const char* ARDF_meters = convert_dBm_to_distance(true_dBm_max);
	const char* ARDF_txpwr_to_mtr = cal_fox_pwr_to_mtr(); 
	//printf("ARDF TX pwr to mtr %s\n",ARDF_txpwr_to_mtr);
	//if (volume < 1 ) volume = 1;	
	if (true_dBm_max <= -520)true_dBm_max = -510;
	dsp_update_params();
		if (keyed)
		freq += p.split_freq;
	if (mode >= MODE_USB && mode <= MODE_LSB) // ARDF MODE_CWL changed to LSB
		freq += p.offset_freq;
	//int true_dBm_max_d = true_dBm_max; // important to change float to int for display.
	//unsigned freq_rounded_kHz = freq / 1000;  // ARDF last 3 digits out
	return snprintf(text, maxlen,
		 "%8lX%4s v37 %lu.%02luV  %2d  %3s   %5s"
		//"1234567890123456789012345678901234567890"		
		//"%6u%2s%4s%4d%4d    %3s",  //40 characters
		,set1,ARDF_meters,BattIntegerPart,BattFractionalPart,p.volume,p_mode_names[mode],ARDF_txpwr_to_mtr
		//,freq_rounded_kHz,ARDF_memory[memARDF],ARDF_meters, true_dBm / 4, (true_dBm_max_d /4 ), p_mode_names[mode]
		);
		
} 

bool att_25dB_ON(bool state) {
	static bool state_prev = false; 
	if(state != state_prev){printf("att_25dB %d\n",state);}
	
    if (state) {
        GPIO_PinOutSet(PA0_25dB_PORT, PA0_25dB_PIN);  // ARDF 25dB attn on
		printf("att_25dB ON %d\n",state);
		//att_25dB_status = true;
    } else {
        GPIO_PinOutClear(PA0_25dB_PORT, PA0_25dB_PIN);  // ARDF 25dB attn off
    }
	state_prev = state;
	att_25dB_status = state;
    return state;  // Return the current state for confirmation
	//printf("25dB attn state %d\n",state);
	
}

bool att_35dB_ON(bool state) {
	static bool state_prev = false; 
	if(state != state_prev){printf("att_35dB %d\n",state);}
	//printf("att_35dB %d\n",state);
    if (state) {
			GPIO_PinOutSet(PA0_25dB_PORT, PA0_25dB_PIN);  // ARDF 25dB attn on
        	//GPIO_PinOutSet(TX_EN_PORT, TX_EN_PIN); // ARDF RX TX ant switch
			//GPIO_PinOutClear(RX_EN_PORT, RX_EN_PIN); 
    } else {
			GPIO_PinOutClear(PA0_25dB_PORT, PA0_25dB_PIN);  // ARDF 25dB attn off
            //GPIO_PinOutClear(TX_EN_PORT, TX_EN_PIN); // ARDF RX TX ant switch
			//GPIO_PinOutSet(RX_EN_PORT, RX_EN_PIN); 
			
    }
	state_prev = state;
	att_35dB_status = state;
    return state;  // Return the current state for confirmation
	//printf("35dB attn state %s\n",state);
	
	}
bool att_65dB_ON(bool state) {
	static bool state_prev = false; 
	if(state != state_prev){printf("att_65dB %d\n",state);}
	//printf("att_65dB %d\n",state);
    if (state) {
        GPIO_PinOutSet(PB12_65dB_PORT, PB12_65dB_PIN);  // F12 65dB attn on
		//att_65dB_status = true;
    } else {
        GPIO_PinOutClear(PB12_65dB_PORT, PB12_65dB_PIN);  // F12 65dB attn off
    }
	state_prev = state;
	att_65dB_status = state;
    return state;  // Return the current state for confirmation
	//printf("65dB attn state %s\n",state);
	
	}



// Function to convert RSSI_read dBm to volume based on a range
float dBm_to_vol(float RSSI_read, float in_min, float in_max, float out_min, float out_max) {
    return (RSSI_read - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



int RSSI_read_desimal() { // ARDF 
	int input = RSSI_read % 4;
    switch (input) {
        case 0:
            return 0;
        case -1:
            return 25;
        case -2:
            return 50;
        case -3:
            return 75;
        default:
            printf("Invalid input: %d. Must be between 0 and -3.\n", input);
            return -1;  // Error code for invalid input
    }
}

void delay_ms(uint32_t ms) {
    uint32_t i;
	printf("waste some time\n");
    for (i = 0; i < ms * 100; i++) {// ARDF Needed for Attn delay Just waste some time
		//printf("waste some time\n");
        __NOP();  // No operation instruction
    }
}

int true_dBm_prev;  // ARDF tallettaa edellisen dBm lukeman
int true_dBm_delay;
int disable_att_65 = 0;
static int counterx = 0;
void adjust_attn(){
	if (flash_read_done == false)ARDF_Read_flash();
		//int global_o_disp_d = global_o_disp;  // ARDF 04052025
		//printf("global o %d \n",global_o_disp); // ARDF 04052025
		//int global_amp_d = global_amp;
		//printf("global_amp: %d \n",global_amp_d);
		if(!att_35dB_status && !att_65dB_status ){
			att_35dB_ON(false);
			att_65dB_ON(false);
			true_dBm = RSSI_read - 13;  // ARDF dBm adjusted to show correct value
            if (true_dBm >= true_dBm_max + 20) true_dBm_max = true_dBm + 60;  // ARDF skaalaa true dbm max isommalle tasolle	
			count_RSSI_ave(true_dBm);
			int true_dBm_d = true_dBm; // changed to integer for print
			//printf(" true dBm: %3d\n", true_dBm_d / 4); // näyttää oikein näytöllä
			global_agc_amp = ((int)dBm_to_vol(true_dBm, true_dBm_max - 100, true_dBm_max, 1000, 300));	// ARDF agc_vol_max
			if(Volume_normal == 0 ) {  // ARDF katso voiko poistaa
			p.volume = (int)dBm_to_vol(true_dBm, -400, true_dBm_max, 100, 45); // ARDF vol plus agc
			if(p.volume < 2) p.volume = 2;
			if(p.volume > 254) p.volume = 254;
			}
			int global_agc_amp_d = global_agc_amp; 
			//printf("Global agc amp attn OFF %d\n", global_agc_amp_d);
			if (true_dBm /4 >= -64){	// ARDF was -68
			global_agc_amp = 80000;  // ARDF adjusting vol down
			p.volume = 0;  // ARDF adjusting vol down
           // printf("Change Attn to 35 ON \n");
			att_35dB_ON(true); // 35dB
			att_65dB_ON(false);
			disable_att_65 = 1;
			true_dBm2 = -200;
			true_dBm_max2 = -200;

		  	 }
		}
		if(att_35dB_status && !att_65dB_status){
			//delay_ms(5000);
			true_dBm2 = RSSI_read + 108;  // ARDF dBm adjusted to show correct value
			if(true_dBm_max2 / 4 >= -20){
				true_dBm2 = RSSI_read + 108;
				true_dBm_max2 = true_dBm2; 
				global_agc_amp = 80000;
				}
			//p.volume = 254;
			//printf("att2 35dB ON \n");
			int true_dBm2_d = true_dBm2; // changed to integer for print
			printf("att 35dB true dBm2: %3d\n", true_dBm2_d / 4); // näyttää oikein näytöllä
            if (true_dBm2 >= true_dBm_max2 ) true_dBm_max2 = true_dBm2 + 60;
			true_dBm = true_dBm2;
			true_dBm_max = true_dBm_max2;
			global_agc_amp = ((int)dBm_to_vol(true_dBm2, true_dBm_max2 - 100, true_dBm_max2, 1000, 200)); // ARDF agc_vol_max 
			if(Volume_normal == 0 ) {  
			p.volume = (int)dBm_to_vol(true_dBm2, -200, true_dBm_max2, 100, 2); // ARDF vol plus agc
			if(p.volume < 2) p.volume = 2;
			if(p.volume > 254) p.volume = 254;
			}
			int global_agc_amp_d = global_agc_amp; 
			printf("Global agc amp attn 35dB %d\n", global_agc_amp_d);
			//printf("agc vol max %d \n",agc_vol_max);
			  	if(RSSI_mov_average <= true_dBm_max -50){
					global_agc_amp += 500; 
				}

			//if(global_agc_amp < 100) audio_tone(200,400);		
    		if ((RSSI_read + 136) / 4 >= -28) {
			printf("attn 65 RSSI_read %d \n", ((RSSI_read + 136) / 4));
			if(disable_att_65 != 1){
				global_agc_amp = 80000;
				p.volume = 0;
    			att_65dB_ON(true);  // 65dB
    			att_35dB_ON(false); // 35dB
				printf("Change Attn to 65 ON \n");
    			delay_ms(20);
				}
			disable_att_65 = 0;
			counterx = 0;
			}


		}
			//true_dBm_prev = true_dBm;	
			//printf("true dBm prev pohja = %d\n",(true_dBm_prev / 4));
		if(att_65dB_status && !att_35dB_status){
			printf("65dB attn ON \n");
			counterx ++;
			if (counterx <= 10){
				 p.volume = 2;
				 global_agc_amp = 80000;
				 }
			if(counterx >= 100){
			printf("Counting %d \n", counterx);	
			true_dBm3 = RSSI_read + 240;
			} 
			//printf("65dB attn true dBm3: %d\n", true_dBm3 / 4); // näyttää oikein näytöllä			
			true_dBm = true_dBm3; // säädetään true dBm oikein
			if (true_dBm3 >= true_dBm_max3) true_dBm_max3 = true_dBm3 + 60;
			true_dBm_max = true_dBm_max3;
			global_agc_amp = ((int)dBm_to_vol(true_dBm3, true_dBm_max3 - 100, true_dBm_max3, 1000, 200)); // ARDF agc_vol_max
			if(Volume_normal == 0 ) {  
			p.volume = (int)dBm_to_vol(true_dBm3, -200, true_dBm_max3, 200, 2); // ARDF vol plus agc
			if(p.volume < 2) p.volume = 2;
			if(p.volume > 254) p.volume = 254; 
			}
			int global_agc_amp_d = global_agc_amp; 
			printf("Global agc amp attn 65dB %d\n", global_agc_amp_d);
			int true_dBm_max_3 = true_dBm_max3; // changed to integer for print
			printf("attn65 true dBm max3: %2d\n", true_dBm_max_3 / 4); // näyttää oikein näytöllä
			counterx = 1000;
            }

			/*if(att_65dB_status && att_35dB_status){
			printf("65dB and 35dB attn ON \n");

			int S_dB_4 = RSSI_read; 
            //int S_dB_3 = (int)((10.0 * log10(rs.smeter))-146); // adjusted to be true dBm
			true_dBm = S_dB_4 + 240; // säädetään true dBm oikein
			if (true_dBm >= true_dBm_max) true_dBm_max = true_dBm;

			global_agc_amp = ((int)dBm_to_vol(RSSI_mov_average, true_dBm_max - 100, true_dBm_max, 1000, agc_vol_max)); // ARDF agc_vol_max 
			  	if(RSSI_mov_average <= true_dBm_max -50){
					global_agc_amp += 500; 
				}
			}*/
}

void count_RSSI_ave(int true_dBm){
	RSSI_mov_average = true_dBm;
}

// Format text specific to FM view
static int ui_view_fm_text(char *text, size_t maxlen)
{	
	int ctcss = ctcss_freqs[ui.ctcss];
	if (ctcss != 0) {
		return snprintf(text, maxlen,
			""  // OH2BTG kenttää säädetty ctcss on 3 rivillä
			//"        %8d%4d.%1d",  // OH2BTG kenttää säädetty ctcss on 3 rivillä
			//p.squelch,
			//ctcss / 10, ctcss % 10
		);
	} else {
		/*return snprintf(text, maxlen,
			"%2d  ",
			p.squelch
		);*/
	}
};

const struct ui_view ui_view_fm = {
	UI_FIELDS_COMMON_N ,
	ui_view_fm_text,
	{
	UI_FIELDS_COMMON,
	//{ UI_FIELD_SQ,       33,34, 2, "Squelch"          },// OH2BTG tässä saa siirrettyä kursorin oikealle paikalle
	//{ UI_FIELD_CTCSS,    36,40, 5, "CTCSS Hz"         },
	}
};


static int ui_view_other_text(char *text, size_t maxlen)
{
	(void)text; (void)maxlen;
	return snprintf(text, maxlen,
		"");
	return 0;

}

const struct ui_view ui_view_other = {
	UI_FIELDS_COMMON_N,
	ui_view_other_text,
	{
	UI_FIELDS_COMMON,
	}
};

struct ui_state ui = {
	.view = &ui_view_fm,
	.cursor = 5,  // ARDF changed cursor to kHz OH2BTG
	.ctcss = 18, //check from ctcss table correct index number 18 = 118.8
};

#define BACKLIGHT_ON_TIME 200
#define BACKLIGHT_DIM_LEVEL 50

#define DISPLAYBUF_SIZE 384
#define DISPLAYBUF2_SIZE 384
uint8_t displaybuf[DISPLAYBUF_SIZE], displaybuf2[DISPLAYBUF2_SIZE];

volatile struct display_ev display_ev;
SemaphoreHandle_t display_sem;

#if DISPLAYBUF_SIZE < 3*8*8
#error "Too small display buffer for text"
#endif

// Wrap number between 0 and b-1
static int wrap(int a, int b)
{
	while(a < 0) a += b;
	while(a >= b) a -= b;
	return a;
}

// Wrap number between -b+1 and b
static int wrap_signed(int a, int b)
{
	while (a <= -b) a += 2*b;
	while (a >   b) a -= 2*b;
	return a;
}

struct ui_text_color {
	// Foreground
	uint8_t fr, fg, fb;
	// Background
	uint8_t br, bg, bb;
};

const struct ui_text_color ui_text_colors[] = {
	{ 0x00, 0x00, 0x00,  0xFF, 0xFF, 0xFF },  // white bg black text
	//{ 0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00 },  // line 0 toka rivi näytössä
	{ 0xFF, 0x00, 0x00,  0xFF, 0xFF, 0xFF },  // line 1 highlight under cursor red = 0xFF, yel = 0xFF,0xFF
	{ 0x00, 0x00, 0xFF,  0xFF, 0xFF, 0xFF },  // white bg black text
	{ 0x00, 0x00, 0xFF,  0xFF, 0xFF, 0xFF },  // white bg black text
	{ 0x00, 0xFF, 0x00,  0xFF, 0xFF, 0xFF },  // white bg black text
	{ 0x00, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF },  // white bg black text
	//	{ 0x00, 0xFF, 0x00,  0x00, 0x00, 0x00 },  // line 2 eka rivi näytöllä grn=fg blk=bg
	//{ 0x00, 0xFF, 0x00,  0x00, 0x00, 0x00 },  // line 3 Taajuusnäytön loppuosa plus mode näytöt
	//{ 0xFF, 0xFF, 0x00,  0xFF, 0x00, 0x00 },  // line 4 Taajuusnäytön loppuosa plus mode näytöt
};


void ui_character(int x1, int y1, unsigned char c, unsigned char color)
{
	int x, y;
	if(!display_ready()) return;

	struct ui_text_color colors = ui_text_colors[color];
	if (c > 0x80)
		c = 0;
	const char *font = font8x8_basic[c];

	display_area(x1, y1, x1+7, y1+7); // first y1 change text row OH2BTG
	display_start();

	uint8_t *bufp = displaybuf;
	for (y=0; y<8; y++) {
		for (x=0; x<8; x++) {
			if (font[y] & (1<<x)) {
				*bufp++ = colors.fr;
				*bufp++ = colors.fg;
				*bufp++ = colors.fb;
			} else {
				*bufp++ = colors.br;
				*bufp++ = colors.bg;
				*bufp++ = colors.bb;
			}
		}
	}
	display_transfer(displaybuf, 3*8*8);
}
void ui_character_big(int x1, int y1, unsigned char c, unsigned char color)
{
 int x, y;
 if(!display_ready()) return;

 struct ui_text_color colors = ui_text_colors[color];
 if (c > 0x80)
  c = 0;
 const char *font = font8x8_basic[c];

 display_area(x1, y1, x1+15, y1+15);  // OH2BTG kokeille
 display_start();

 for (y=-1; y<8; y++) {  // OH2BTG kokeilee y=-1 siirtää kirjaimen alun näkyviin
  uint8_t *bufp = displaybuf;
  for (x=0; x<8; x++) {
   if (font[y] & (1<<x)) {
    *bufp++ = colors.fr;
    *bufp++ = colors.fg;
    *bufp++ = colors.fb;
    *bufp++ = colors.fr;
    *bufp++ = colors.fg;
    *bufp++ = colors.fb;
   } else {
    *bufp++ = colors.br;
    *bufp++ = colors.bg;
    *bufp++ = colors.bb;
    *bufp++ = colors.br;
    *bufp++ = colors.bg;
    *bufp++ = colors.bb;
   }
  }
  // Repeat each row of pixels twice
  for (x=0; x<2; x++) {
   display_transfer(displaybuf, 3*16);
  }
 }
}

void ui_update_text(void) {
	int r;
	// TODO: put signal strength back somewhere
	//int s_dB = 10.0*log10(rs.smeter);

	unsigned cursor = ui.cursor;
	const struct ui_view *view = ui.view;

	char *textbegin = ui.text;
	char *text = textbegin;
	size_t maxlen = TEXT_LEN + 1;
	//ARDF_disp_on = 1;
	if(ARDF_disp_on == 1){
		r = ui_view_ARDF_text(text, maxlen);
		printf("ARDF_disp_on \n");
		}else{
	r = ui_view_common_text(text, maxlen);
	}
	text += r;
	maxlen -= r;

	r = view->text(text, maxlen);
	text += r;
	maxlen -= r;

	// Fill the rest with spaces
	for (; text < textbegin + 50; text++, maxlen--)  // OH2BTG was 48
		*text = '  ';

	r = snprintf(text, maxlen,
		"BTG",// %s  ARDF ei enää jumita näyttöä 
		view->fields[cursor].tip
	);
	text += r;
	maxlen -= r;

	for (; maxlen > 0; text++, maxlen--)
		*text = ' ';

	size_t n, i;
	for (i = 0; i < TEXT_LEN; i++) {
		ui.color[i] = 0;
	}
	for (n = 0; n < view->n; n++) {
		char c = 0;
		if (cursor == n) {
			// Highlight cursor
			c = 1;
		} else {
			c = view->fields[n].color;
		}
		size_t pos1 = view->fields[n].pos1;
		size_t pos2 = view->fields[n].pos2;
		for (i = pos1; i <= pos2; i++)
			ui.color[i] = c;
	}
}

static void ui_choose_view(void)
{
	switch (p.mode) {
	case MODE_ARDF:
	printf("Mode ARDF");
		break;
	case MODE_FM:
		ui.view = &ui_view_fm;
		break;
	case MODE_USB:
		break;
	case MODE_LSB:
	//case MODE_CWU:
	//case MODE_CWL:
	//	ui.view = &ui_view_ssb;
		break;
	default:
		ui.view = &ui_view_other;
		//ui.view = &ui_view_fm;
	}
}
//mem_prev = 0;
int case_prev = 0; // ARDF
const int initf0 = 144503000, initf1 = 144763000, initf2 = 145625000, initf3 = 145500000 , initf4 = 144469000;
int mem0 = initf0, mem1 = initf1, mem2 = initf2, mem3 = initf3, mem4 = initf4; 
mem5 = 1248;
static void ARDF_fill_mem(){
    switch (case_prev) {
        case 0: mem0 = mem_prev; break;
        case 1: mem1 = mem_prev; break;
        case 2: mem2 = mem_prev; break;
        case 3: mem3 = mem_prev; break;
        case 4: mem4 = mem_prev; break;
		//case 5: mem5 = mem_prev; break;
        default: 
            printf("Warning: Invalid case_prev value: %d\n", case_prev);
            break;
    }
}
	 void ARDF_Read_flash(){
			printf("Read flash data...\n");
			printf("Verifying saved data...\n");
			flash_read_done = true;
    		uint32_t read_value;
    		for (uint32_t i = 0; i < num_values; i++) {
        	read_value = *((uint32_t*)(USERDATA_BASE + i * sizeof(uint32_t)));  // Read back data
			//printf("Recovered Value after pwr off %u: %u\n", i, read_value);
        	printf("Recovered Value after pwr off %u: %u\n", i, read_value);
			        switch (i) {
            case 0: mem0 = read_value * 1000; break;
            case 1: mem1 = read_value * 1000; break;
            case 2: mem2 = read_value * 1000; break;
            case 3: mem3 = read_value * 1000; break;
            case 4: mem4 = read_value * 1000; break;
			case 5: set1 = read_value;	
			Convert_set1();
			break;

        }


    } 
}

void Convert_set1(void){
unsigned int hex_value = set1; 
set1_last_bit = hex_value & 0xF;
printf("set1  0x%X\n",hex_value);
printf("last_bit_desimal %u\n",set1_last_bit);
ADJ_ARDF_fox_pwr = set1_last_bit;
}

static void ARDF_ui_choose_view(memARDF) {
    printf("ARDF_ui_choose_view, Case = %d\n", memARDF);
	if (flash_read_done == false)ARDF_Read_flash();
    // Update memory and frequency based on the selected case
    switch (memARDF) {
        case 0:
            mem_prev = p.frequency;
            ARDF_fill_mem();
            printf("Case 0\n");
        	if(mem0 <= 13000000){
			p.frequency = initf0;
			printf("mem0 =%d\n",mem0);
			} 
			else {p.frequency = mem0;
			}
			xSemaphoreGive(railtask_sem);
            case_prev = 0;
            break;

        case 1:
            mem_prev = p.frequency;
            ARDF_fill_mem();
            printf("Case 1\n");
            if(mem1 <= 13000000){
			p.frequency = initf1;
			printf("mem1 =%d\n",mem1);
			} 
			else {p.frequency = mem1;
			}
			xSemaphoreGive(railtask_sem);
            case_prev = 1;
            break;
		case 2:
            mem_prev = p.frequency;
            ARDF_fill_mem();
            printf("Case 2\n");
            if(mem2 <= 13000000){
			p.frequency = initf2;
			printf("mem2 =%d\n",mem2);
			} 
			else {p.frequency = mem2;
			}
			xSemaphoreGive(railtask_sem);
            case_prev = 2;
            break;
		case 3:
            mem_prev = p.frequency;
            ARDF_fill_mem();
            printf("Case 3\n");
            if(mem3 <= 13000000){
			p.frequency = initf3;
			printf("mem3 =%d\n",mem3);
			} 
			else {p.frequency = mem3;
			}
			xSemaphoreGive(railtask_sem);
            case_prev = 3;
            break;
		case 4:
            mem_prev = p.frequency;
            ARDF_fill_mem();
            printf("Case 4\n");
            if(mem4 <= 13000000){
			p.frequency = initf4;
			printf("mem4 =%d\n",mem4);
			} 
			else {p.frequency = mem4;
			}
			xSemaphoreGive(railtask_sem);
            case_prev = 4;
            break;
			
        // Add more cases if needed
        default:
            printf("Invalid memARDF value: %d\n", memARDF);
            break;
    }
}	

//static const int ui_steps[] = { 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9 };
static const int ui_steps[] = { 1e2,1e3, 1e4, 1e5, 1e6, 1e7, 1e8 }; // ARDF locked some freq adj steps
static void ui_knob_turned(enum ui_field_name f, int diff) 
{
	if (/*f >= UI_FIELD_FREQ0 && */f <= UI_FIELD_FREQ7) {  // ARDF locked 3 last steps out
		p.frequency += diff * ui_steps[UI_FIELD_FREQ7 - f]; // ARDF locked 3 last steps out
		//mem_prev = p.frequency;
		//printf("Kaava mem_prev frequency%10d\n ", mem_prev);
		xSemaphoreGive(railtask_sem);
		
	}
	else if (f == UI_FIELD_MTR) {
		//ADJ_ARDF_fox_pwr = wrap(ADJ_ARDF_fox_pwr + diff, sizeof(ARDF_distance) / sizeof(ARDF_distance[0]));
		//printf("ADJ ARDF dist%d \n", ADJ_ARDF_fox_pwr);

		ADJ_ARDF_fox_pwr = wrap(ADJ_ARDF_fox_pwr + diff, sizeof(ARDF_Pout_mtr) / sizeof(ARDF_Pout_mtr[0]));
		int max_index = sizeof(ARDF_Pout_mtr) / sizeof(ARDF_Pout_mtr[0]) - 1;

		//if(ADJ_ARDF_fox_pwr <=0) ADJ_ARDF_fox_pwr = 0;
		//if(ADJ_ARDF_fox_pwr >=15) ADJ_ARDF_fox_pwr = 15;
		printf("ADJ ARDF fox pwr %d \n", ADJ_ARDF_fox_pwr);
		ARDF_scale_fox_pwr = ADJ_ARDF_fox_pwr - 11; // 11 equals 200m
		//if(ARDF_scale_fox_pwr <= -11)ARDF_scale_fox_pwr = -11;
		//if(ARDF_scale_fox_pwr >= 4)ARDF_scale_fox_pwr = 4;
		
		printf("ARDF_scale_fox_pwr %d\n",ARDF_scale_fox_pwr);

		//dsp_update_params();
		//ARDF_ui_choose_view();
		ui_choose_view();
	}
	else if (f == UI_FIELD_MODE) {
		p.mode = wrap(p.mode + diff, sizeof(p_mode_names) / sizeof(p_mode_names[0]));
		dsp_update_params();
		ui_choose_view();
	}
	else if (f == UI_FIELD_MEM){
		memARDF = wrap(memARDF + diff, sizeof(ARDF_memory) / sizeof(ARDF_memory[0]));
		//dsp_update_params();
		ARDF_ui_choose_view(memARDF);
		//printf("UI FIELD ARDF selected\n");

	}
	//else if (f == UI_FIELD_PTT) {
	//	ui.keyed = wrap(ui.keyed + diff, 10);
	//}




	/*
	else if (f == UI_FIELD_VOL) {
		Volume_normal = 1;
		p.volume = wrap(p.volume + diff, 254); // OH2BTG Vol max
		if(p.volume > 254)p.volume = 254;
		if(p.volume < 0)p.volume = 0;	
		dsp_update_params();
	}
	//else if (f == UI_FIELD_WF) {
	//	p.waterfall_averages = wrap(p.waterfall_averages + diff, 100);
	//}
	/*
	else if (f == UI_FIELD_SQ) {
		p.squelch = wrap(p.squelch + diff, 100);
		dsp_update_params();
	}*/
	
	
	else if (f == UI_FIELD_WF_S_METER) {
		set_ARDF_ON = true;
		int size = sizeof(Wfall_S_Meter) / sizeof(Wfall_S_Meter[0]);
		Wf_Smeter += diff;
		if (Wf_Smeter < 0) { // SMeter
        Wf_Smeter = 0;
		}
		else if (Wf_Smeter > 2) { // Waterfall
        Wf_Smeter = 2;
    } 
		/*printf("Current selection: %s\n", call_Wfall_SMeter(Wf_Smeter)); // 
		if(Wf_Smeter == 2){
		printf("ARDF mode on");	
		}*/
	}
	

	
}

	void draw_color_block(int x, int y, int width, int height, uint8_t r, uint8_t g, uint8_t b){
    // Step 1: Define the rectangular area for the block
    display_area(x, y, x + width - 1, y + height - 1);

    // Step 2: Start the drawing process
    display_start();

    // Step 3: Fill the block with the specified color
    for (int i = 0; i < width * height; i++) {
        display_pixel(r, g, b); // Set each pixel to the specified RGB color
    }
    // Step 4: End the drawing process
    display_end();
	}

// count only every 4th position
#define ENCODER_DIVIDER 4
uint32_t Saved_value;
uint32_t Cleared_value;	// ARDF for memory handling
uint32_t Set_value;	// ARDF for memory handling
/* ui_check_buttons is regularly called from the misc task.
 * TODO: think about thread safety when other tasks
 * read the updated data */



void ui_check_buttons(void){
	int pos_now, pos_diff,pos_diff1; // ARDF pos_diff tells if encoder is turned 
	char button = get_encoder_button();// pwr_on = get_pwr_on;// ptt = get_ptt(); 
	char ardf_F11 = get_ardf_F11_button(), ardf_pa5 = get_ardf_pa5_button();
	static int ardf_F11_press_count = 0;
	static int button_press_count = 0;
	static int button_press_count2 = 0; // käytetään TST menussa
	pos_now = get_encoder_position() / ENCODER_DIVIDER;
	pos_diff1 = pos_now - ui.pos_prev;
	pos_diff = 0;

		if(ardf_pa5){
		Volume_normal = 1;	
		pos_diff1 = 0;
		pos_diff = pos_now - ui.pos_prev;
		}else {
		Volume_normal = 0;	
		}
		

		if(pos_diff1 == 1 ){			//ARDF rotary SW +
		printf("ARDF and Pos diff1 + %d\n",pos_diff1);
		ARDF_disp_on = 0;
		ui_update_text();
		//true_dBm = true_dBm + 10;
		p.volume = p.volume +10;
		//att_25dB_ON(true); // 25dB
		}
		if(pos_diff1 == -1 ){		//ARDF rotary SW -
		printf("ARDF and Pos diff1  %d\n",pos_diff1);
		//p.volume = p.volume -10;
		ARDF_disp_on = 0;
		ui_update_text();
		//att_25dB_ON(false); // 25dB

		}
		

		if (p.mode == MODE_OFF && ui.button_prev && (!button))  {
			// Shut down after button has been released.
			printf("MODE OFF \n");
			button_press_count++;
			if(button_press_count == 2){
			printf("ARDF MODE OFF Button pressed %d \n", button_press_count);
			shutdown();
			button_press_count = 0;
			}
	
		}
		//if (p.mode == MODE_TST && ui.button_prev && (!button)&& (ardf_pa5)) {
		if (p.mode == MODE_TST && ui.button_prev && (!button) && (ardf_pa5)) {
			button_press_count2++;
			if(button_press_count2 == 1){
			ARDF_disp_on = 1;
			//ui_update_text;
			printf("ARDF_disp ON \n");
			}else if(button_press_count2 == 2){
			ARDF_disp_on = 0;
			ui_update_text;
			printf("ARDF_disp OFF \n");
			//button_press_count2 = 0;
			}
			else if(button_press_count2 == 3){
			ARDF_disp_on = 1;
			ui_update_text;
			printf("ARDF_disp ON count 3 \n");
			button_press_count2 = 0;
			}
		}
		if (p.mode == MODE_ARDF && ui.button_prev && (!button)&& ardf_pa5) {// Tallettaa muistiin MEM näytöllä				
			printf("MODE ARDF \n");
			ARDF_disp_on = 0;
			ui_update_text();
			// ARDF memory write exercise
  			// Chip errata
  			CHIP_Init();
  			// Declare the value to be stored in flash
			uint32_t data_to_write[] = {mem0/1000, mem1/1000, mem2/1000, mem3/1000, mem4/1000,set1}; // Values to write
    		num_values = sizeof(data_to_write) / sizeof(data_to_write[0]);
			printf("Num values %u \n",num_values);
			//printf("Data to write %u \n",data_to_write);
			for (uint32_t i = 0; i < num_values; ++i) {
   			printf("data_to_write[%u] = %u\n", i, data_to_write[i]);
			}
			if (num_values * sizeof(uint32_t) > USERDATA_SIZE) {
    		printf("Error: Data size exceeds USERDATA memory size!\n");
    		return;
			}				
  			//Clear the Userdata page of any previous data stored
  			MSC_ErasePage(USERDATA);
			int erase_status = MSC_ErasePage(USERDATA);
			    if (erase_status != 0) {
        	printf("Flash erase failed with error code: %d\n", erase_status);
        	return;
			}
  			// Write the value into the Userdata portion of the flash
	  		MSC_Init();
			for (uint32_t i = 0; i < num_values; i++) {
        	MSC_WriteWord((uint32_t*)(USERDATA + i), &data_to_write[i], sizeof(data_to_write[i]));
    		}
  			MSC_Deinit();//
			printf("Read flash data...\n");
			printf("Verifying saved data...\n");
    		uint32_t read_value;
    		for (uint32_t i = 0; i < num_values; i++) {
        	read_value = *((uint32_t*)(USERDATA_BASE + i * sizeof(uint32_t)));  // Read back data
        	printf("Recovered Value when write %u: %u\n", i, read_value);
    		}
 		}



		if (ui.button_prev && (!button)&& (!ardf_pa5)) { // OH2BTG ARDF changes between Fox and beacon Freq
			//ARDF_Read_flash();
			ARDF_disp_on = 0;
			ui_update_text;
			button_press_count++;
			if(button_press_count == 2){
			memARDF = 0;
			ARDF_ui_choose_view(1);
			xSemaphoreGive(railtask_sem);		
			}else if(button_press_count == 4){
			memARDF = 1;
			ARDF_ui_choose_view(0);
			xSemaphoreGive(railtask_sem);
			button_press_count = 0;
			}
		}
		if (ardf_F11 && (!ui.ardf_F11_prev)){ // ARDF attn off F11 Vipukytkin palautuva asento 
			ARDF_disp_on = 0;
			ui_update_text;
			//ardf_F11_press_count++; // Increment the press count
		    //if (ardf_F11_press_count == 1) {
        	//true_dBm_max = true_dBm_max - 100; // First press: subtract 20
			//global_agc_amp = 80000;
			true_dBm_max = -500;    // ARDF Variable to hold result (float)
			true_dBm_max2 = -500;
			true_dBm_max3 = -500;
			//dsp_update_params();
			//if(att_65dB_status) {
			att_65dB_ON(false);	
			att_35dB_ON(false);
			att_25dB_ON(false);
			//}
			disable_att_65 = 1;
			adjust_attn();
			//}
			// att_65dB_ON(false);
			// delay_ms(5000);
			global_agc_amp = 80000;
			//p.volume = p.volume + 100;
			//printf("ARDF F11 Button pressed 1 time\n");
			} 
			
	

		if (ardf_pa5 && (!ui.ardf_pa5_prev)){ // ARDF MENU PA5 switch ON
			// put here all experimental stuff
			printf("ARDF PA5 Switch pressed\n");
		}

		if (ardf_pa5 && Wf_Smeter == 2 && ui.button_prev && (!button)&&set_ARDF_ON ){ // ARDF MENU PA5 ON Wf_Smeter = 2BTG and button true 
			// put here all experimental stuff
			button_press_count++;
			if(button_press_count == 2){
			//printf(" 2BTG ON \n");
			Volume_normal = 1;
			p.volume = 254;
			agc_normal = 1;	// Not ARDF mode, normal AGC 
			dsp_update_params();
			ardf_ON_OFF = 0;
			Wf_Smeter = 4;	
			}
			else if(button_press_count == 4){
			//printf(" 2BTG OFF \n");
			Volume_normal = 0;// ARDF RSSI change volume
			p.volume = 20;
			agc_normal = 0; // ARDF mode
			dsp_update_params();	
			ardf_ON_OFF = 1;
			button_press_count = 0;
			//set_ARDF_ON = false; 
			Wf_Smeter = 3; 
			}
			
			
		}

	const struct ui_view *view = ui.view;

		if (button){
			if(ARDF_disp_on == 1){
			ARDF_disp_on = 0;
			//ui_update_text;	
			}  
			ui.backlight_timer = 0;
		}
		if (pos_diff) {
		if(pos_diff >= 0x8000 / ENCODER_DIVIDER)
			pos_diff -= 0x10000 / ENCODER_DIVIDER;
		else if(pos_diff < -0x8000 / ENCODER_DIVIDER)
			pos_diff += 0x10000 / ENCODER_DIVIDER;
		if (button) { // ARDF OH2BTG set PA5 switch as setup mode
			printf("button pressed \n");
			ui.cursor = wrap(ui.cursor + pos_diff, view->n);
		} else {
			size_t c = ui.cursor;
			if (c < view->n) {
				ui_knob_turned(view->fields[c].name, pos_diff);
			}
		}
		ui.backlight_timer = 0;
		}
		/*if (pos_diff != 0 || ptt != ui.ptt_prev) {
		if (tx_freq_allowed(p.frequency + p.split_freq)) {
			//p.keyed = ui.keyed || ptt;  // ARDF disable transmitter
			p.keyed = 0; // ARDF disable transmitter
		} else {
			p.keyed = 0;
			ui.keyed = 0;
		}
		//if (p.keyed != ui.keyed_prev)// ARDF disable transmitter
		//	xSemaphoreGive(railtask_sem);// ARDF disable transmitter
		//ui.keyed_prev = p.keyed; //ARDF disable transmitter
		// Something on the display may have changed at this point,
		// so make the display task check for that. 
		display_ev.text_changed = 1;
		xSemaphoreGive(display_sem);
		}*/

	ui.pos_prev = pos_now;
	//ui.ptt_prev = ptt;
	ui.button_prev = button;
	ui.ardf_F11_prev = ardf_F11;
	ui.ardf_pa5_prev = ardf_pa5;

}

void BattV_read(){
		read_batt_adc();
		uint32_t readBatt = readADC;
		if(readBatt <= 10){
		read_batt_adc();
		readBatt = readADC;
		}
		printf("ADC Value: %lu\n", readBatt); // Print ADC result
		//uint32_t BattVoltage = ((readBatt + 10) * 36) /10000;

 		uint32_t fullVoltage = ((readBatt + 10) * 36); // Full voltage scaled by 10000
    	BattIntegerPart = fullVoltage / 10000;   // Integer part of the voltage
    	BattFractionalPart = (fullVoltage % 10000)/100; // Remainder gives the decimal part 2 digits

		// Print integer and fractional parts
    	printf("Battery Voltage = %lu.%02lu V\n", BattIntegerPart, BattFractionalPart);

}


/* ui_control_backlight is regularly called from the misc task. */
void ui_control_backlight(void)
{
	if (ui.backlight_timer <= BACKLIGHT_ON_TIME) {
		display_backlight(BACKLIGHT_DIM_LEVEL + BACKLIGHT_ON_TIME - ui.backlight_timer);
		ui.backlight_timer++;
	}
}


int fftrow = FFT_ROW2;
#if DISPLAYBUF2_SIZE < 3*(FFT_BIN2-FFT_BIN1)
#error "Too small display buffer for FFT"
#endif

/* Check for the waterfall line flag and draw the line.
 * If the flag is not set, just return. */
static void ui_display_waterfall(void){
	if (!display_ev.waterfall_line)
		return;
	display_ev.waterfall_line = 0;
	if (!display_ready()) {
	//	printf("Bug? Display not ready in waterfall\n");
		return;
	}
	display_scroll(fftrow);
	display_area(0,fftrow, FFT_BIN2-FFT_BIN1, fftrow);
	display_start();
	display_transfer(displaybuf2, 3*(FFT_BIN2-FFT_BIN1));

	fftrow--;
	if(fftrow < FFT_ROW1) fftrow = FFT_ROW2;

}



static void ARDF_ui_display_waterfall(void) { // ARDF 080202025
    if (offset_cursor_ready == 0)
        return;
    offset_cursor_ready = 0;
    
    if (!display_ready()) {
        return;
    }
    
    display_scroll(fftrow);
    display_area(0, fftrow, 128, 20); // ARDF 09022025 20
    display_start();

    // Define vol_line properly
    uint8_t vol_line[128 * 3] = {0}; // Initialize to black

    // Compute the volume bar position
	//int x = (1 / (global_agc_amp )* 1000);
    //int x = volume * 2;
    //if (x < 0) x = 0;
    //if (x > 128) x = 128; // Prevent overflow
	int bar_width = 128 - ((global_agc_amp - 70) * 127) / (2000 - 70); // ARDF
	if (bar_width < 1) bar_width = 1;
	if (bar_width > 127) bar_width = 127;



   // printf("volume = %d \n", volume);
   // printf("x = %d \n", x);

    // Fill the volume bar with color
    for (int i = 0; i < bar_width; i++) {
        vol_line[i * 3] = RSSI_bar_R;      // Red
        vol_line[i * 3 + 1] = RSSI_bar_G; // Green
        vol_line[i * 3 + 2] = RSSI_bar_B;   // Blue
    }

    // Send to display
    display_transfer(vol_line, sizeof(vol_line));

    fftrow--;
    if (fftrow < FFT_ROW1) 
        fftrow = FFT_ROW2;
}


static void RSSI_to_color(void){// ARDF 19012025

	if(RSSI_mov_average/4 <= -100){ // green
	 RSSI_bar_R = 70;
	 RSSI_bar_G = 255;
	 RSSI_bar_B = 100;	
	}
	if(RSSI_mov_average/4 >= -100 && RSSI_mov_average/4 <= -80){ // yellow
	 RSSI_bar_R = 245;
	 RSSI_bar_G = 255;
	 RSSI_bar_B = 80;	
	}
	if(RSSI_mov_average/4 >= -80 && RSSI_mov_average/4 <= -60){ // orange
	 RSSI_bar_R = 255;
	 RSSI_bar_G = 230;
	 RSSI_bar_B = 70;	
	}
	if(RSSI_mov_average/4 >= -60 && RSSI_mov_average/4 <= -40){ // red
	 RSSI_bar_R = 255;
	 RSSI_bar_G = 70;
	 RSSI_bar_B = 180;	
	}
	if(RSSI_mov_average/4 >= -40 && RSSI_mov_average/4 <= -30){ // pink
	 RSSI_bar_R = 255;
	 RSSI_bar_G = 130;
	 RSSI_bar_B = 255;	
	}
	if(RSSI_mov_average/4 >= -30 && RSSI_mov_average/4 <= -20){  // blue
	 RSSI_bar_R = 180;
	 RSSI_bar_G = 180;
	 RSSI_bar_B = 255;	
	}
	if(RSSI_mov_average/4 >= -20 ){ // white
	 RSSI_bar_R = 255;
	 RSSI_bar_G = 255;
	 RSSI_bar_B = 255;	
	}
	
}

/* Draw the offset frequency cursor above waterfall */
void ardf_draw_volume(void){ // ARDF draw Volume / RSSI bar 
 //	static int clear_counter = 0; // Persistent counter across function calls

	RSSI_to_color();
	//int bar_width = 1 + volume * 2;
	//int bar_width = 1 + 1 / global_agc_amp * 2500;
	//int bar_width =  (1 / (global_agc_amp) * 5000);
	int bar_width = 128 - ((global_agc_amp - 70) * 127) / (2000 - 10); // ARDF
	if (bar_width < 1) bar_width = 1;
	if (bar_width > 127) bar_width = 127;

    draw_color_block(bar_width, 80, 127, 16, 0, 0, 0); // Clear the whole bar area
	draw_color_block(0, 80, bar_width, 16, RSSI_bar_R, RSSI_bar_G, RSSI_bar_B);
	   // **Only clear the display every 10 calls**

    // Draw the stored blocks
    /*for (int i = 0; i < color_block_count; i++) {
        draw_color_block(color_blocks[i].x, color_blocks[i].y,
                         color_blocks[i].width, color_blocks[i].height,
                         color_blocks[i].r, color_blocks[i].g, color_blocks[i].b);
	}*/
	
	offset_cursor_ready = 1;

}



/* Update text on the display.
 *
 * To make both the text and the waterfall respond fast
 * for smooth user experience, draw the text one character
 * at a time and check for a possible new waterfall line
 * in between drawing each character.
 * Also update only the characters that have changed. */



static void ui_display_text1(void)  // ARDF version
{
	ui_update_text();
	int i;
	for (i = 0; i < TEXT_LEN; i++) {
		char c = ui.text[i],  cp = ui.textprev[i];
		unsigned char v = ui.color[i], vp = ui.colorprev[i];
		if (c != cp || v != vp) {

			if (i < 8) // first line
				ui_character_big(i*16, 0, c, v);
			else if (i <16) // second line
				ui_character_big(((i-8)*16),16, c, v);  // i define where text start
			else if(i < 24) // test end of 2nd line small fonts for dBm
				ui_character_big(((i-16)*16), 32, c, v);  
			else if(i < 32) // third line FM RX  vol sql
				ui_character_big((i-24)*16, 48, c, v);  // 
			else if(i < 40) // forth line FM and mode specific 
				ui_character_big((i-32)*16, 64, c, v);  // v = 2 = blue 1 = red you can change colour
			else  //bottom line OH2BTG
				ui_character(((i-40)*16),80, c, v); 

			ui.textprev[i] = c;
			ui.colorprev[i] = v;

			// ui_display_waterfall();  // E10 OH2BTG
		
		}
	}
	// ardf_draw_volume();  // ARDF draw volume bar
	// ardf_draw_volume_stored();
}


void display_task(void *arg)
{
	(void)arg;
	display_init();
	for (;;) {
		xSemaphoreTake(display_sem, portMAX_DELAY);
		if(Wf_Smeter == 0){
		ui_display_waterfall(); // Waterfall 08022025 ARDF OH2BTG test 22062025
		}
		if(Wf_Smeter == 1){
		ARDF_ui_display_waterfall(); // Smeter 08022025  ARDF test 22062025
		}
		if(Wf_Smeter == 2||3||4){
		ui_display_waterfall(); //07072025 ARDF test as "ARDF"	
		}
			ardf_draw_volume();
		if (display_ev.text_changed) {
			display_ev.text_changed = 0;
			ui_display_text1();
		}
	}
}


/* Create the RTOS objects needed by the user interface.
 * Called before starting the scheduler. */
void ui_rtos_init(void)
{
	display_sem = xSemaphoreCreateBinary();
}
