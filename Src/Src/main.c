/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "dma.h"
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "quadspi.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fmc.h"

/* USER CODE BEGIN Includes */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		This firmware for F7 chip, 32F746G DISCOVERY board.
//		Added support for some Rekordbox functions, display of static and dynamic waveform, audio interpolation process. 
//
//		Added support for control buttons with led backlight.
//		IMPORTANT!!! 
//		After regenerating the code with Cube, make these corrections:
//		File sai.c
//		 - hsai_BlockA2.FrameInit.ActiveFrameLength = 32;
//		File dma2d.c, comment 2 lines:
//		 - //  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
//		 - //  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
//		File arm_math.h, add line:
//		 - #define ARM_MATH_CM7
//		init this functions after LCD_Layers initialization:	
//			-	MX_DMA_Init();
//  		-	MX_SPI1_Init();
//			-	HAL_SPI_TransmitReceive_DMA(&hspi1, Tbuffer, Rbuffer, 27);
//
//			File \F7 XDJ_PANEL\Drivers\BSP\STM32746G-Discovery\stm32746g_discovery_lcd.c
//				- //HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);      //300 line
//
//
//
//
//		Compilator settings:	
//
//--c99 -c --cpu Cortex-M7.fp.sp -D__MICROLIB -g -O1 --apcs=interwork --split_sections -I../Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I..\Drivers\BSP\STM32746G-Discovery -I..\Drivers\BSP\Components\rk043fn48h -I..\Drivers\BSP\Components\Common -I..\Utilities\CPU -I..\Utilities\Fonts -I..\Utilities\Log -I../Middlewares/Third_Party/FatFs/src/drivers -I../Middlewares/Third_Party/FatFs/src --C99 
//-I "C:\Keil_v5\My_Project\F7 XDJ_PANEL\MDK-ARM\RTE" 
//-I C:\Keil_v5\ARM\PACK\ARM\CMSIS\4.5.0\CMSIS\Include 
//-I C:\Keil_v5\ARM\PACK\Keil\STM32F7xx_DFP\2.11.0\Drivers\CMSIS\Device\ST\STM32F7xx\Include 
//-D__UVISION_VERSION="517" -D_RTE_ -DSTM32F746xx -DUSE_HAL_DRIVER -DSTM32F746xx -o "F7\*.o" --omf_browse "F7\*.crf" --depend "F7\*.d" 
//
//		generated by:
//			STM32CubeMX 4.22.0
//			STM32Cube FW_F7 V1.7.0
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	ver. 0.61a
//		- optimized browser function
//		- added dark_green color in browser for text on white line cursor 
//	ver. 0.62a
//		- optimized DATABASE_PARSER finction
//		- DATABASE_PARSER using WFORMDYNAMIC massive only 
//	ver. 0.63a
//		- optimized LOAD_TRACK function. Now the PCM[] massive is not used.
//		- PCM[] converted in uint16_t
//		- optimized audio processing for new uint16_t PCM[]. Now the calculation of 1 sample is 5us instead of 7.5us 
//	ver. 0.64a
//		- fixed joint in circular buffer in audio processing
//		- optimized ring buffer size for increased audio processing performance
//	ver. 0.65a
//		- optimized audio processing (read SDRAM). Now the calculation of 1 sample is 4us 
//	 	- added SD_LED blink
//		- added encoder blink when loading track
//	ver. 0.66a
//		- added touch screen control
//	ver. 0.67a
//		- added basic function needle search
//		- fixed function BSP_LCD_DisplayStringAt. 
//		-	added TRANSPARENT_MODE for text. 
//		- changed method of determination the font with dynamic width (Font15P) and other fonts with static width.  
//		- optimized touch controller driver
//		- create audioparcer for SEEK function
//	ver. 0.68a
//		- added QSPI flash support
//		- added animation startup logo
//	ver. 0.69a
//		- added REKORDBOX logo on startup
//		- added tag list 
//		- added internal function for RedrawWaveforms process 
//	ver. 0.70a
//		- added INFO mode in BROWSER and TAG LIST menu
//		- added internal functions for optimize code
//	ver. 0.71a
//		- improved the functionality of the TAG LIST menu
//	ver. 0.72a
//		- added symbol red check mark for TAG LIST
//		- improved the functionality of the TAG LIST (added TAG TRACK/REMOVE button)
//		- bugs fixes internal functions browser and tag list
//		- bugs fixes NAVIGATOR function
//		- added loading track from TAG LIST
//	ver. 0.73a
//		- added UTILITY window
//		- added long press MENU button for entering to UTILITY
//		- fixed check WAV header
//		-	optimized process add/delete tracks in TAG LIST
//	ver. 0.74a
//		- added SPI transfer for CDJ-1000mk3 panel
//	ver. 0.75a
//		- added reading cues and memory pionts from ANLZXXXX.DAT
//		- fixed bug SPI DMA (see note*) 
//	ver. 0.76a
//		- added PLAY, TEMPO, TEMPO RESET, JOG MODE buttons
//		- added pitch slider 4 ranges
//		- added calculation tempo and bpm after pitch change
//	ver. 0.77a
//		- added PLAY blink led, CUE led
//		- improved jog vinyl mode (inertial process)
//		- added TRACK SEARCH buttons
//		- improved ShowTempo function
//		- added TIME MODE button, REMAIN MODE
//		- added CDJ JOG MODE work
//		- optimized pitch bend coeficients aka CDJ-1000mk3
//		- optimized VINYL MODE precision jog step
//	ver. 0.78a
//		- improved static scroll UI
//		- added BEATGRID massive
//		- shift up 1px static information, static waveform 
//		- beatgrid support
//		- improved Phase Meter
//	ver. 0.79a
//		- improved BPM calculating after pitch change
//		- added BPMGRID for tracks with variable BPM 
//	ver. 0.80a
//		- added Font13D for phase bars
//		- added slip mode marker on jog display
//		- added master player ICON
//		- added slip mode red button and slip mode jog illumination
//		- fixed bug spin jog at maximum speed (variable overflow)
//		- added first SLIP MODE functions with audio processing
//		- added animation icon and gradient for phase bars
//		- improved jog pitch bend in reverse mode
//		- improved jog in slip mode
//	ver. 0.81a
//		- added CUE blink
//		- improved time mode button code
//		- added VINYL RELEASE/START and TOUCH/BREAKE mode 
//		-	optimized SPI-DMA transfer process
//		-	optimized potenciometer's curve for VINYL RELEASE/START and TOUCH/BREAKE mode
//	ver. 0.82a
//		- added track nubmer and status (playing or played) in INFO mode
//		- added filling buffer step sequencer for optimize time gaps
//		- fixed filling buffer step sequencer algoritm
//	ver. 0.83a
//		- improved the work of the function of static and dynamic waveforms
//		- added "remain/foward time style" for progress bar
//	ver. 0.84b
//		- added blink progress bar when the remaining time is less than 30sec
//		- fixed DrawMinuteMarkers function
//		- improved performance static and dynamic waveforms (added ForceDrawVLine function)
//		- added checking device UID 
//		- fixed fatal error when deleting a track from an empty tag list
//		- the first addition of a function CUE audio
//	ver. 0.85b
//		- added CUE button process
//		- improved CUE audio process
//		- optimized pitch bend coefficients
//		- added loading of Hot Cues and Memory Cues attributes
//	ver. 0.86b
//		- added MEMORY CUE calling
//		- bugs fixes calling CUEs when jog in CDJ mode
//		- added CUE, MEMORY CUE and HOT CUE triangles on dynamic waveform
//	ver. 0.90b
//		- database parser DeviceSQL updated based on document: "Rekordbox Export Structure Analysis" James Elliott Deep Symmetry, LLC
//		- change colorystic dynamic waveform (lower white point)
//		- change long touch timer for MENU button
//		- added bpm to INFO menu
//		- added colored rating to INFO menu
//		- added KEY to INFO menu
//		- added duration to INFO menu
//		- maximum tracks in database - 512 (not enough memory)
//		- maximum playlists in database - 20 (not enough memory)
//		- supported only latin encoding in track names and tags
//		- added encoder signal filter
//		- show KEY on waveform fisplay
//		- added pages in the browser with animation: playlists, tracks, SD card information
//		- browser animation bug fixed
//		- resized cue marker on dynamic waveform
//		- change logic INFO button
//		- fixed database parser
//		- fixed text line overflow error
//		- added REALTIME CUE (set, when track playing) 			
//		- added AUTO CUE to the first bit of the bitgrid
//		- added full UID chip in HEX in utility			
//		- fixed TAG LIST exiting borders when deleting tracks
//	ver. 0.97b
//		- improved browser menu
//		- add flash disk name and date (at the root of the browser)
//		- fixed floating pitch tempo values
//		-	improved slip mode on CUE
//		- fixed the work of jog with a hot CUE
//		-	improved mechanical imitation of jog
//		-	changed color and style gradient bar
//		-	changed color dynamic waveform
//		- optimized dynamic waveform work
//		- added QUANTIZE for CUE and LOOP
//		- exclude noise at the end of the track
//		- added CRC control for SPI Rx package
//		- fixed start phase detection BEATGRID
//		-	added LOOP MODE (beta)
//		- optimized time display function to improve performance	
//	ver. 1.03
//		-	changed function control for SPI Rx package
//		- in UTILITY it is now possible to change parameters and save to internal memory
//		- added load lock
//		- added AUTO CUE LEVEL with MEMORY, FIRST BEAT MODE and analog tresholds (-36dB, -42dB and other)
//		-	added TIME MODE DEFAULT 
//		-	added TEMPO RANGE DEFAULT 
//		-	SLIP REVERSE MODE for REVERSE SWITCH
//		-	added ability to select RGB or BLUE waveforms
//		-	added LCD brightness
//		-	added jog indicator when the track ends 
//		-	added jog brightness
//		-	added BPM color
//		-	added audio output level
//		- for use JOG PWM output pin CN4-7 for it to work. see schematic STM32F746 Discovery
//		- for LCD PWM to work, remove R85 and install 
//			it between U10-7 and GND. Remove R81 and R66. Connect together the U10-7 and RMII_CRS_DV, 
//			at the point where R66 was.) see schematic STM32F746 Discovery
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////		

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "ff.h"
#include "wm8994.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#include <math.h>
#include "arm_math.h"
#include "skins.h"
#include "logo.h"
#include "ft5336.h"
#include "stm32746g_discovery_qspi.h"
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;

///////////////////////////////////////////FatFS///////////////////
unsigned int nbytes;
FRESULT res;
FIL file;
FILINFO finfo;
FATFS FAT;
//FATFS * fs;
DIR directory;
uint32_t free_mem = 0;
uint32_t used_mem = 0;

/* Dynamic Waveform variables ---------------------------------------------------------*/
uint32_t all_long = 0;						//all long of Track in 0.5*frames   150 on 1 sec
uint8_t DynamicWaveformZOOM = 4;	//zoom parametr 1-2-4-8-16
char WFORMDYNAMIC[135000]; 		//__attribute__ ((section(".sdram")));; for force waveform
uint32_t BEATGRID[4096];			// beatgrid (0, 3, 7... )
uint16_t BPMGRID[4096];				// bpmgrit BPM*100
uint8_t GRID_OFFSET = 1;			//ofsset grid 1.2.3.4
uint32_t PreviousPositionDW = 0;
uint8_t Prev10m = 0xFF;
uint8_t Prev1m = 0xFF;
uint8_t Prev10s = 0xFF;
uint8_t Prev1s = 0xFF;
uint8_t Prev10f = 0xFF;
uint8_t Prev1f = 0xFF;
uint8_t PrevHf = 0xFF;
uint8_t VisibleLayer = 0;
#define BROWSER 			0
#define WAVEFORM 			1
#define BROWSER_INFO 	2
#define TAG_LIST			3
#define TAG_LIST_INFO 4
#define UTILITY				5
#define BROWSER_NAVI	6
#define BR_NAVI_END		7
uint8_t forcibly_redraw = 0; 
uint8_t RED_VERTICAL_LINE = 0;
uint16_t PreviousPhase = 0;
uint16_t bars = 0;						//
#define LOOP_ACTIVE_COLOR					0xFFB05B00
#define LOOP_INACTIVE_COLOR 			0xFF313131
#define CUE_COLOR									0xFFF08138

const uint32_t COLOR_MAP[2][8] = 
{
0xFF063878,
0xFF1C4A87,
0xFF39679A,
0xFF5A85AF,
0xFF7BA2C4,
0xFF9BC0D8,
0xFFB8DAEA,
0xFFDFF2F8,
0xFFD80000,			//C90000	
0xFFBB2149,
0xFFEC4D58,
0xFFF0A767,
0xFFA7CD24,
0xFF55BB3D,
0xFF3ED1C9,
0xFF3D87DC	
};


/* Static Waveform variables ---------------------------------------------------------*/
uint16_t previous_position_bar = 0;
char WFORMSTATIC[400];
uint8_t dSHOW = BROWSER;
uint8_t MemoryCuePyramid_ENABLE = 0;
#define DRAW_NEW_STATIC_WAVEFORM		400
#define CLEAR_WAVEFORM_ARRAY				401
#define MS_NOT_LOADED								402
#define REDRAW_IN_NREMAIN_MODE			403
#define REDRAW_IN_REMAIN_MODE				404
#define MS_ERROR										410
uint8_t DRAWN_IN_REMAIN = 0;
uint8_t need_DSW = 0;						//flag for redraw progressbar remain or nrmain mode when track long 30s
#define PBAR_COLOR_1				((uint32_t)0xFFC4C5C6)
#define PBAR_COLOR_2				((uint32_t)0xFF64656D)
#define PBAR_COLOR_3				((uint32_t)0xFF44454A)


const uint32_t WS_COLOR_MAP[2][2][8] = 
{
0xFF5D799E,
0xFF5D799E,	
0xFF5D799E,	
0xFF5D799E,	
0xFF5D799E,
0xFF5D799E,	
0xFF5D799E,	
0xFF5D799E,		
0xFF9DC7ED,
0xFF9DC7ED,
0xFF9DC7ED,
0xFF9DC7ED,
0xFF9DC7ED,
0xFF9DC7ED,
0xFF9DC7ED,
0xFF9DC7ED,	
	
0xFFF0A767,
0xFFFFC499,	

0xFFBB2146,
0xFFD85E97,

0xFFEC4D58,
0xFFCF4214,

0xFFC90000,
0xFFEF0000,

0xFF3D87DC,
0xFF53AEE2,

0xFF3ED1C9,
0xFF53F3EA,

0xFF55BB3D,
0xFF28D100,

0xFFA7CD24,
0xFFEFED26
};








/* HOT CUES and MEMORY variables ---------------------------------------------------------*/
#define NONE_MARK				0
#define MEMORY_MARK			1
#define HOT_CUE_A_MARK	2
#define HOT_CUE_B_MARK	3
#define HOT_CUE_C_MARK	4
uint32_t HCUE_adr[2][3] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};  																		//[0] => HCUE in 0.5*frames [1] => loop end in 0.5*frames   150 on 1 sec;  
uint32_t MEMORY_adr[2][10] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
															0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}; 	//[0] => MEMORY CUE in 0.5*frames [1] =>  MEMORY loop end in 0.5*frames   150 on 1 sec;
uint8_t	HCUE_type[3] = {0};	 			//b0 (0=cue / 1=loop); b1 (0=inactive / 1=active); 													
uint8_t	MEMORY_type[10] = {0};		//b0 (0=cue / 1=loop); b1 (0=inactive / 1=active); 

uint8_t number_of_hot_cue_points = 0;
uint8_t number_of_memory_cue_points = 0;

/* Audio processing variables ---------------------------------------------------------*/
uint32_t play_adr = 0;						//Playing adress in samples (44100 per second)
uint32_t slip_play_adr = 0;				//Playing adress for SLIP MODE in samples (44100 per second)
uint32_t sdram_adr = 0;
uint8_t SAMPLE[4];
uint16_t PCM[206][8192][2]					__attribute__ ((section(".sdram")));;
uint16_t start_adr_valid_data = 0;							//filling adress in memory
uint16_t end_adr_valid_data = 0;								//filling adress in memory ()
uint8_t filling_step = 0;
uint8_t reload = 1;
uint8_t change_speed = 0;							//flag for RELEASE/START or TOUCH/BREAKE 
#define NO_CHANGE	0
#define NEED_UP		1
#define NEED_DOWN	2
uint32_t CUE_ADR = 0;					//REAL CUE adr in frames 150
uint8_t offset_adress = 0;			//address offset for calling CUE audio data (for work)
uint8_t mem_offset_adress = 0;	//address offset for calling CUE audio data (for memory)

uint16_t acceleration_UP = 0xFFFF;				//acceleration_UP for vinyl RELEASE/START
uint16_t acceleration_DOWN = 0xFFFF;			//acceleration_DOWN for vinyl TOUCH/BREAKE
uint16_t LOG_TABLE[8] = {1200, 128, 72, 40, 18, 15, 9, 3};		

uint16_t PCM_2[2];
int16_t LR[2][4];
float c0, c1, c2, c3, r0, r1, r2, r3;
int32_t even1, even2, odd1, odd2;
static float COEF[8] = {			//////optimal 2x
0.45868970870461956,
0.04131401926395584,
0.48068024766578432,
0.17577925564495955,
-0.246185007019907091, 
0.24614027139700284,
-0.36030925263849456,
0.10174985775982505
};
uint32_t i = 0;
uint8_t reverse = 0;
uint16_t pitch = 10000;	// 10000 = 100% step 0,01%			
uint32_t position = 0;
uint32_t slip_position = 0;
uint16_t pitch_for_slip = 10000;	// 10000 = 100% step 0,01%	
uint8_t step_position = 0;
float SAMPLE_BUFFER;
float T;
uint8_t QUANTIZE = 1;					//QUANTIZE ENABLE
uint8_t	end_of_track = 0;			//end track flag
uint8_t loop_active = 0;			//loop flag
uint32_t LOOP_OUT = 0;				//adr LOOP OUT in frames 150
uint8_t lock_control = 1;			//lock buttons, when track not loading or in process

/* Rekordbox processing variables ---------------------------------------------------------*/
uint8_t playlist[512][55] 		__attribute__ ((section(".sdram")));;		//54byte - status byte
uint32_t parcser_adress[512] 	__attribute__ ((section(".sdram")));;
uint16_t original_tempo[512] 	__attribute__ ((section(".sdram")));;
uint16_t rating[512] 	__attribute__ ((section(".sdram")));;			//rating and color tracks
uint16_t duration[512] 	__attribute__ ((section(".sdram")));;
uint8_t key_id[512] 	__attribute__ ((section(".sdram")));;
char SDCARD_NAME[20] = "SD CARD";
char SD_DATE[15] = "23-01-2020";
uint16_t TOTAL_TRACKS = 0;
uint16_t TOTAL_TRACKS_IN_CURRENT_PLAYLIST = 0;
uint16_t TRACK_PLAY_IN_CURRENT_PLAYLIST = 0;
uint16_t track_play_now = 0;
char path_export[]="0:/PIONEER/rekordbox/export.pdb";
char KEYS[25][4];
const uint32_t COLOR_MAP_RATING[9] = 
{
0xFF404040,
0xFFF870F8,
0xFFF80000,
0xFFF8A030,
0xFFF8E330,
0xFF00E000,
0xFF00C0F8,
0xFF0050F8,
0xFF9808F8,	
};
uint16_t TRACKS_DATABASE[1024];		//database track ID [playlist 1[ID][ID][ID]][playlist 2[ID][ID][ID]][playlist 3[ID][ID][ID]]....
uint8_t TRACKLIST_NAME[20][21];		//20 tracklists max and 21 lengt
uint16_t TRACKLIST_OFFSET[20];		//offset for TRACKS_DATABASE
uint8_t TOTAL_TRACKLISTS = 0;			//maximum 20 tracklists



/* Display and static information variables ---------------------------------------------------------*/
#define LCD_FRAME_BUFFER    SDRAM_DEVICE_ADDR
#define LCD_FRAME_BUFFER_2	((uint32_t)0xC0780800)
char FIRMWARE_VERSION[] = "1.03";		
uint8_t lcd_status = LCD_OK;
uint8_t Buf[64]={0};
uint16_t originalBPM = 0xFFFF;						//this original BPM*100 of track (pitch = 0.00%) 
uint8_t REMAIN_ENABLE = 1;
uint32_t BLUE_BAR[4][14] = {
    0xffe5e5fb, 0xffe5e5fb, 0xffe5e5fb, 0xffe5e5fb, 0xffe2e2fb, 0xffdcdcfa, 0xffd5d5f8, 0xffc9c9f6, 0xffbbbbf4, 0xffaaaaf2, 0xff9696ef, 0xff8181ec, 0xff6c6ce8, 0xff5757e5, 
    0xffe2e2fb, 0xffe1e1fb, 0xffe0e0fb, 0xffddddf9, 0xffd9d9f9, 0xffd1d1f8, 0xffc8c8f7, 0xffbcbcf5, 0xffadadf2, 0xff9c9cf0, 0xff8888ee, 0xff7474ea, 0xff6060e6, 0xff4d4de4, 
    0xffc0c0f5, 0xffbfbff5, 0xffbcbcf5, 0xffb7b7f4, 0xffafaff3, 0xffa7a7f2, 0xff9b9bf0, 0xff8e8eee, 0xff8080ec, 0xff7171e9, 0xff6060e7, 0xff5050e4, 0xff4040e2, 0xff3232e1, 
    0xff7878ea, 0xff7878ea, 0xff7575ea, 0xff7070e9, 0xff6a6ae8, 0xff6363e7, 0xff5959e6, 0xff5050e5, 0xff4646e3, 0xff3a3ae2, 0xff3030df, 0xff2626de, 0xff1d1ddc, 0xff1414db
};
uint32_t RED_BAR[4][14] = {
    0xfffae3df, 0xfffae3df, 0xfffae3de, 0xfffae2dd, 0xfff9dfdb, 0xfff8dbd7, 0xfff6d5ce, 0xfff6ccc3, 0xfff3bfb5, 0xfff0b0a5, 0xffed9f92, 0xffea8e7d, 0xffe77d68, 0xffe46a55, 
    0xfff9dfdb, 0xfff9dfdb, 0xfff9deda, 0xfff8dcd7, 0xfff8d8d3, 0xfff6d2cc, 0xfff5cac2, 0xfff4c0b6, 0xfff1b3a8, 0xffefa497, 0xffec9484, 0xffe88371, 0xffe5725d, 0xffe3624b, 
    0xfff4c2ba, 0xfff4c2b9, 0xfff3c0b6, 0xfff3bbb1, 0xfff1b6aa, 0xfff0ada2, 0xffefa597, 0xffed998a, 0xffea8d7c, 0xffe87f6d, 0xffe5725d, 0xffe3644d, 0xffe0573e, 0xffde4b30, 
    0xffe98775, 0xffe98674, 0xffe88472, 0xffe8806c, 0xffe77b66, 0xffe67460, 0xffe46d57, 0xffe2654e, 0xffe15c44, 0xffe05339, 0xffdd492e, 0xffdc4125, 0xffda391b, 0xffd93113
};


/* Display Browser variables ---------------------------------------------------------*/
uint8_t B0CurrentCursorPosition = 0;			//0...7 position
uint8_t B1CurrentCursorPosition = 0;		//0...7 position
uint8_t B2CurrentCursorPosition = 0;		//0...7 position
uint16_t BCurrentPlaylistPosition = 1;			//1....TOTAL_TRACKLISTS-7
uint16_t BCurrentTrackPosition = 1;			//1....TOTAL_TRACKS-7
uint8_t ScrollLong = 142;								//5...142
uint8_t ScrollPosition = 0;							//0...142-ScrollLong
#define LCD_COLOR_DGREEN           	((uint32_t)0xFF00CD00)
#define LCD_COLOR_LIGHT_1           ((uint32_t)0xFFDFDFDF)				//color for line-cursor browser
#define LCD_COLOR_LIGHT_2           ((uint32_t)0xFFEFEFEF)				//color for line-cursor browser
#define LCD_COLOR_LIGHT_3						((uint32_t)0xFFCFCFCF)				//color for secondary line-cursor UTILITY
#define LCD_COLOR_LIGHT_4						((uint32_t)0xFFAFAFAF)				//color for secondary line-cursor UTILITY
#define LCD_COLOR_LIGHT_5						((uint32_t)0xFFBFBFBF)				//color for secondary line-cursor UTILITY
#define LCD_COLOR_DARK_1           	((uint32_t)0xFF606060)
#define LCD_COLOR_DARK_2           	((uint32_t)0xFF404040)
#define BROWSER0_DOWN		0
#define BROWSER0_UP			1
#define TAGLIST_DOWN		2
#define TAGLIST_UP			3
#define UTILITY_DOWN		4
#define UTILITY_UP			5
#define BROWSER1_DOWN		6
#define BROWSER1_UP			7
#define BROWSER2_DOWN		8
#define BROWSER2_UP			9

/* Display TAG LIST variables ---------------------------------------------------------*/
uint8_t TOTAL_TRACKS_IN_TAG_LIST = 0;					//0....100
uint16_t TAG_LIST_BASE[101] = {0};				//base tag list [track number]
uint8_t TCurrentCursorPosition = 0;			//0...7 position
uint8_t TCurrentTrackPosition = 1;			//1....TOTAL_TRACKS_IN_TAG_LIST-7
uint8_t s = 0;													//counter for write-delete-add tag track

/* Display INFO variables ---------------------------------------------------------*/
uint8_t BROWSER_INFO_enable = 0;
uint8_t TAG_LIST_INFO_enable = 0;
#define LCD_COLOR_PAPER           	((uint32_t)0xFFFFFCE3)
#define LCD_COLOR_SHADOW           	((uint32_t)0xFFD4D4D4)

/* Display UTILITY variables ---------------------------------------------------------*/
uint8_t TOTAL_U_POSITIONS = 15;			
uint8_t UTILITY_SETTINGS[13]; 
uint8_t UTILITY_SETTINGS_MAX[13] = {1, 1, 9, 1, 3, 1, 1, 1, 4, 1, 4, 	1, 9}; 
char UTILITY_BASE[15][20] = {"PLAY MODE           ",
														 "LOAD LOCK           ",
														 "AUTO CUE LEVEL      ",
														 "TIME MODE DEFAULT   ",
														 "TEMPO RANGE DEFAULT ",
														 "SLIP FLASHING       ",
														 "SWITCH REVERSE MODE ",
														 "COLOR WAVEFORM      ",
														 "LCD BRIGHTNESS      ",
														 "JOG INDICATOR       ",
														 "JOG BRIGHTNESS      ",
														 "BPM COLOR						",
														 "OUTPUT LEVEL        ",
														 "DEVICE UID          ",
														 "VERSION No.         "};				//UTILITY list
uint32_t DEVICE_UID = 0;
uint8_t UCurrentCursorPosition = 0;			//0...7 position
uint8_t CurrentUPosition = 1;			//1....TOTAL_UTILITY_POSITIONS-7
uint8_t	countUTILITY = 0;
uint8_t edit_parameter = 0;								//flag in settings menu - entering to edit parameter														 
uint8_t JOG_BRIGHTNESS[5]={1, 11, 40, 109, 245};														 
uint8_t need_rewrite_flash = 0;				
#define UTILITY_START_ADDR   0x080C0000						//sector 7 
uint8_t U;

/* Display Browse ANIMATION variables ---------------------------------------------------------*/														 
uint32_t animation_time = 0;
uint8_t animation_en = 0;				//0 - none ro end, 1 - foward, 2 - back
uint16_t animation_step = 0; 														 
uint16_t previous_animation_step = 0; 
#define LCD_COLOR_PAPER_TRANSP           	((uint32_t)0xAFFFFCE3)
uint8_t animation_finish = 1;														 
uint8_t info_animation_enable = 0;	
uint8_t BROWSE_LEVEL = 2;								//0 - tracklist, 1 - playlists, 2 - menu playlist, filename 3 - SD card information														 
uint8_t PREVIOUS_BROWSE_LEVEL = 2;			//
uint8_t CURRENT_LAY;

														 
/* Buttons variables ---------------------------------------------------------*/
uint8_t need_up = 0;
uint8_t need_down = 0;
uint8_t LOAD_pressed = 0;
uint8_t KEY_BROWSE_pressed = 0;
uint8_t KEY_SD_pressed = 0;
uint8_t KEY_TAG_LIST_pressed = 0;
uint8_t KEY_BACK_pressed = 0;
uint8_t KEY_MENU_pressed = 0;
uint8_t KEY_TAG_TRACK_pressed = 0;
uint8_t	KEY_USB_pressed = 0;
uint8_t KEY_MIDI_pressed = 0;														 
uint8_t KEY_INFO_pressed = 0;
uint8_t LED_SD_timer = 0;
uint8_t timer_time = 0;
uint8_t ENCODER_LED_BLINK = 8;
uint32_t encoder_tim = 0;		 

/* Touch screen variables ---------------------------------------------------------*/
static TS_StateTypeDef TS_State;
static uint8_t tscnt[2]={0};
#define TS_NEEDLE_X_MIN		40
#define TS_NEEDLE_X_MAX		439
#define TS_NEEDLE_Y_MIN		235
#define TS_NEEDLE_Y_MAX		271
uint8_t needle_enable = 0;
uint16_t previous_needle_position = 0;

/* SPI transfer variables ---------------------------------------------------------*/
uint8_t Tbuffer[27] = {
	168,  119,  119,  0,  119,  119,  0,  0,  0,  1,  176,  0,  0,  0,  0,  0,  0,  0xC,  0,  0x20,  0,  0,  0,  88,  0,  0,  0};
uint8_t load_animation_enable = 0;
uint8_t Rbuffer[27]={0};
uint16_t zi = 0;
uint8_t a = 0;
uint8_t dma_cnt = 0;
uint32_t DD;
uint16_t potenciometer_tempo;
uint32_t pitch_center;
uint32_t previous_adc_pitch;
uint32_t previous_adc_DD;
uint16_t previous_potenciometer_tempo = 0;
uint8_t tempo_need_update = 0;
uint8_t tempo_range = 1;					//10% default
uint8_t tempo_range_need_update = 0;
uint8_t time_mode_need_update = 0;
uint8_t quantize_mode_need_update = 0;
uint8_t track_need_load = 0;
uint8_t play_enable = 0;
uint8_t slip_play_enable = 0;
uint8_t inertial_rotation = 0;							//inertial rotation for jog
uint8_t keep_to_play = 0;
uint8_t PLAY_BUTTON_pressed = 0;
uint8_t CUE_BUTTON_pressed = 0;
uint8_t JOG_MODE_BUTTON_pressed = 0;
uint8_t TEMPO_RESET_BUTTON_pressed = 0;
uint8_t TEMPO_RANGE_BUTTON_pressed = 0;
uint8_t TRACK_NEXT_BUTTON_pressed = 0;
uint8_t TRACK_PREVIOUS_BUTTON_pressed = 0;
uint8_t CALL_NEXT_BUTTON_pressed = 0;
uint8_t CALL_PREVIOUS_BUTTON_pressed = 0;
uint8_t SEARCH_FF_BUTTON_pressed = 0;
uint8_t SEARCH_REW_BUTTON_pressed = 0;
uint8_t TIME_MODE_BUTTON_pressed = 0;
uint8_t QUANTIZE_BUTTON_pressed = 0;							//TEXT MODE real button
uint8_t SLIP_MODE_BUTTON_pressed = 0;
uint8_t REVERSE_SWITCH_pressed = 0;
uint8_t REALTIME_CUE_BUTTON_pressed = 0;
uint8_t LOOP_OUT_BUTTON_pressed = 0;	
uint8_t RELOOP_BUTTON_pressed = 0;
uint8_t TIM_PLAY_LED = 0;
uint8_t TIM_CUE_LED = 0;
uint8_t RED_CIRCLE_CUE_ADR = 1;
uint8_t TMPSLP = 0;
uint8_t REALTIME_CUE_LED_BLINK = 16;
uint8_t TIM_REALTIME_CUE_LED = 1;
uint8_t LOOP_LEDS_BLINK = 0;
uint8_t JOG_PRESSED = 0;
uint8_t need_call_to_cue = 0;
uint8_t keep_slip = 0;

uint8_t CUE_OPERATION = 0;
#define CUE_NEED_SET 1
#define CUE_NEED_CALL 2
#define MEMORY_NEED_NEXT_SET 3
#define MEMORY_NEED_PREVIOUS_SET 4
#define MEMORY_NEED_SET_PART2 5




uint8_t	ftp = 0;										/////temp!
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void GET_SD_INFO(void);								//check used and free memory on sd card
void DrawMinuteMarkers(void);
void DrawZOOMGRID(void);
void DrawREKORDBOX(void);
void DrawLOGO(void);
void DrawStaticWFM(uint16_t Tpos);
void DrawTempoRange(uint8_t Range);
void DrawMemoryCuePyramid(uint8_t draw);
void ShowBPM(uint16_t bpm);
void ShowACUE(uint8_t acue);
void ShowREMAIN(uint8_t remain);
void ShowQUANTIZE(uint8_t color);					//0 - none 1 - gray 2 - red
void DrawStaticInformation(void);
void ShowNEEDLE(uint8_t needle);
void ShowTempo(uint16_t tempo);
void ShowTrackNumber(uint8_t track);
void RedrawWaveforms(uint32_t position);
void DrawCueMarker(uint16_t p);														//Draw orange triangle CUE marker
void DrawMemoryMarker(uint16_t p, uint8_t type, uint32_t color);							//Draw red triangle memory marker
void DAC_Init_and_Start(uint8_t volume);
uint16_t DATABASE_PARSER(void);
uint16_t LOAD_TRACK(uint16_t TRACK_NUMBER);
void PREPARE_LOAD_TRACK(uint16_t TRACK_NUMBER, uint16_t TRACK_IN_PLAYLIST);
void ShowPhaseMeter(uint16_t phase);											//Show Phase meter 0 - none 1...4 phase
void SwitchInformationLayer(uint8_t LAY);									//Switch Dynamic Waveform/Browser
void ReDrawScroll(uint16_t total_elements, uint16_t current_element_pos);
void NAVIGATOR(uint8_t UPDOWN);														//Navigate browser, TAG LIST and UTILITY	
void int_B_DRAW_ALL_LINES(void);													//internal function for Browser
void int_B_DRAW_ONE_LINE(uint8_t UPDOWN);									//internal function for Browser
void int_BIx_DRAW_ALL_LINES(uint8_t lvl);									//internal function for Browser + INFO
void int_BI_DRAW_ONE_LINE(uint8_t UPDOWN);								//internal function for Browser + INFO
void int_T_DRAW_ALL_LINES(void);													//internal function for Browser
void int_T_DRAW_ONE_LINE(uint8_t UPDOWN);									//internal function for Browser
void int_TI_DRAW_ALL_LINES(void);													//internal function for Browser + INFO
void int_TI_DRAW_ONE_LINE(uint8_t UPDOWN);								//internal function for Browser + INFO
void int_U_DRAW_ALL_LINES(void);													//internal function for Browser
void int_U_DRAW_ONE_LINE(uint8_t UPDOWN);									//internal function for Browser
void int_U_REDRAW_ONE_LINE(void);													//internal function for UTILITY for change parameter
void int_reload_parameter(void);													//apply new parameters
void int_reload_parameter_realtime(void);									//apply new parameters realtime
void int_B1_DRAW_ONE_LINE(uint8_t UPDOWN);								//internal function for Browser 1 level
void int_B2_DRAW_ONE_LINE(uint8_t UPDOWN);								//internal function for Browser 2 level
void int_DRAW_STARS_RATING(uint16_t rat);									//internal function for Browser + INFO 
uint8_t PlaylistID_to_Pos(uint8_t ID);										//convert playlist ID to position in Tracklist name
void intDrawLayer0_ANIMATION(uint8_t CurrentCursorPosition);	//internal function for Browser animation finish
void intDrawLayer0_INFO_ANIMATION(uint8_t CurrentCursorPosition);
void intDrawLayer0_NOINFO_ANIMATION(uint8_t CurrentCursorPosition);
void intDrawLayer0_BROWSER_1_3(uint8_t CurrentCursorPosition);
void intDrawTriangle(uint8_t CurrentCursorPosition);			//draw triangle for browser with INFO
void int_VALUE_to_KEY(uint8_t val);												//internal function for show KEY 
void intDrawLayer0_INFO(uint8_t CurrentCursorPosition);		//draw layer 0 for INFO BROWSER and TAGLIST
void intDrawLayer0_NOINFO(uint8_t CurrentCursorPosition);	//draw layer 0 for without INFO BROWSER and TAGLIST
void intDRAW_WAVEFORM_FRAME(uint32_t position);						//internal function for redraw waveform
void int_DRAW_TRANSPARENT_BAR(void);											//internal function transparent bar for Browser, waveform
void TOUCH_SCREEN_HANDLER(void);													//touch screen handler
void SEEK_AUDIOFRAME(uint32_t seek_adr);									//seek adress in samples (44100 per second)
void UTILITY_PARAMETER(uint8_t num_parameter);						//write to Buf[] name state parameter for utility
void CheckTXCRC(void);
uint8_t CheckRXCRC(void);
void SET_CUE(uint32_t nf_adr);															//nf_adr = new frame adress in 1/150s
void CALL_CUE(void);
void SET_MEMORY_CUE_1(uint32_t nf_adr);
void SET_MEMORY_CUE_2(void);

void SAI2_IRQHandler(void)												////////////////////////////////AUDIO PROCESSING   44K1Hz//////////////////////////////
	{
	//HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_SET);
	HAL_SAI_IRQHandler(&hsai_BlockA2);
	HAL_SAI_Transmit_IT(&hsai_BlockA2, SAMPLE, 2);
	
		
		if(((play_adr+step_position+3)<=(294*all_long)))						//change all_long extract!
			{
			end_of_track = 0;	
			}
		else
			{
			end_of_track = 1;		
			}			
		
		
	if(Tbuffer[19]&0x8 && ((slip_play_adr+((slip_position+pitch_for_slip)/10000))<(294*all_long)) && slip_play_enable)					//SLIP MODE ENABLE
		{
		slip_position+= pitch_for_slip;
		slip_play_adr+=slip_position/10000;	
		slip_position = slip_position%10000;	
		}
		
	position+= pitch;
	
	if(position>9999)	
		{
		step_position = position/10000;				
		if(reverse==0 && end_of_track==0)					
			{			
			play_adr+= step_position;	
			if(step_position==1)
				{
				LR[0][0] = LR[0][1];
				LR[1][0] = LR[1][1];
				LR[0][1] = LR[0][2];
				LR[1][1] = LR[1][2];
				LR[0][2] = LR[0][3];
				LR[1][2] = LR[1][3];					
				}
			else
				{
				sdram_adr = play_adr&0xFFFFF;						
				LR[0][0] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];							
				LR[1][0] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];
				sdram_adr = (play_adr+1)&0xFFFFF;
				LR[0][1] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];								
				LR[1][1] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];		
				sdram_adr = (play_adr+2)&0xFFFFF;
				LR[0][2] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];									
				LR[1][2] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];
				}
			sdram_adr = (play_adr+3)&0xFFFFF;	
			LR[0][3] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];							
			LR[1][3] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];		
			}
		else if(reverse==1 && play_adr>=step_position)
			{
			play_adr-= step_position;
			if(step_position==1)
				{
				LR[0][0] = LR[0][1];
				LR[1][0] = LR[1][1];
				LR[0][1] = LR[0][2];
				LR[1][1] = LR[1][2];
				LR[0][2] = LR[0][3];
				LR[1][2] = LR[1][3];
				sdram_adr = (play_adr)&0xFFFFF;	
				LR[0][3] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];							
				LR[1][3] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];		
				}
			else
				{
				sdram_adr = play_adr&0xFFFFF;						
				LR[0][3] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];							
				LR[1][3] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];
				sdram_adr = (play_adr+1)&0xFFFFF;
				LR[0][2] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];								
				LR[1][2] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];		
				sdram_adr = (play_adr+2)&0xFFFFF;
				LR[0][1] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];									
				LR[1][1] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];
				sdram_adr = (play_adr+3)&0xFFFFF;	
				LR[0][0] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][0];							
				LR[1][0] = PCM[(sdram_adr>>13)+offset_adress][sdram_adr&0x1FFF][1];			
				}	
			}	
		position = position%10000;	
		}	

	T = position;
	T = T/10000;
	T = T - 1/2.0F;
	
	even1 = LR[0][2];
	even1 = even1 + LR[0][1];
	odd1 = LR[0][2];
	odd1 = odd1 - LR[0][1];
	even2 = LR[0][3];
	even2 = even2 + LR[0][0]; 
	odd2 = LR[0][3];
	odd2 = odd2 - LR[0][0];
	c0 = (float)even1*COEF[0];
	r0 = (float)even2*COEF[1];
	c0 = c0 + r0;
	c1 = (float)odd1*COEF[2];
	r1 = (float)odd2*COEF[3];
	c1 = c1 + r1;
	c2 = (float)even1*COEF[4]; 
	r2 = (float)even2*COEF[5];
	c2 = c2 + r2;
	c3 = (float)odd1*COEF[6];
	r3 = (float)odd2*COEF[7];
	c3 = c3 + r3;

	SAMPLE_BUFFER = c0+T*(c1+T*(c2+T*c3));
	SAMPLE_BUFFER = SAMPLE_BUFFER*0.90F;
	PCM_2[0] = (int)SAMPLE_BUFFER;

	even1 = LR[1][2];
	even1 = even1 + LR[1][1];
	odd1 = LR[1][2];
	odd1 = odd1 - LR[1][1];
	even2 = LR[1][3];
	even2 = even2 + LR[1][0]; 
	odd2 = LR[1][3];
	odd2 = odd2 - LR[1][0];
	c0 = (float)even1*COEF[0];
	r0 = (float)even2*COEF[1];
	c0 = c0 + r0;
	c1 = (float)odd1*COEF[2];
	r1 = (float)odd2*COEF[3];
	c1 = c1 + r1;
	c2 = (float)even1*COEF[4]; 
	r2 = (float)even2*COEF[5];
	c2 = c2 + r2;
	c3 = (float)odd1*COEF[6];
	r3 = (float)odd2*COEF[7];
	c3 = c3 + r3;

	SAMPLE_BUFFER = c0+T*(c1+T*(c2+T*c3));
	SAMPLE_BUFFER = SAMPLE_BUFFER*0.90F;
	PCM_2[1] = (int)SAMPLE_BUFFER;
	
	SAMPLE[3] = PCM_2[0]/256;
	SAMPLE[2] = PCM_2[0]%256;
	SAMPLE[1] = PCM_2[1]/256;
	SAMPLE[0] = PCM_2[1]%256;
	//HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_RESET);
	}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_SAI2_Init();
  MX_TIM1_Init();
  MX_QUADSPI_Init();
  MX_TIM12_Init();
  MX_TIM14_Init();

  /* USER CODE BEGIN 2 */
	BSP_QSPI_Init();
  BSP_QSPI_MemoryMappedMode();
	WRITE_REG(QUADSPI->LPTR, 0xFFF);
	
	CheckTXCRC();
	
	Touch_Ini();
	
	BSP_SD_Init();
		
	lcd_status = BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_2);
	
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(0x00000000);				//верхний слой залит прозрачным цветом
	BSP_LCD_SetTransparency(1, 0);		//слой выставлен полностью прозрачным
	
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTransparency(0, 255);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	
	MX_DMA_Init();
  MX_SPI1_Init();
	HAL_Delay(500);	
	HAL_SPI_TransmitReceive_DMA(&hspi1, Tbuffer, Rbuffer, 27);
	
	for(U=0;U<15;U++)		
		{
		UTILITY_BASE[U][19] = 0;
		}	
			
	for(U=0;U<13;U++)		
		{
		if(((volatile uint8_t*)(UTILITY_START_ADDR))[U]>UTILITY_SETTINGS_MAX[U])
			{
			need_rewrite_flash = 1;	
			}
		else
			{
			UTILITY_SETTINGS[U]	= ((volatile uint8_t*)(UTILITY_START_ADDR))[U];					//flash -> ram utility
			}			
		}	
		
	if(need_rewrite_flash)					//apply default settings
		{
		UTILITY_SETTINGS[0] = 0;			//PLAY MODE: 0 - single*; 1 - continue
		UTILITY_SETTINGS[1] = 0;			//LOAD LOCK:	0 - UNLOCK*; 1 - LOCK
		UTILITY_SETTINGS[2] = 9;			//AUTO CUE LEVEL:  0 -36dB; 1 -42dB; 2 -48dB; 3 -54dB; 4 -60dB; 5 -66dB; 6 -72dB; 7 -78dB; 8 - MEMORY; 9 - FIRST BEAT*;
		UTILITY_SETTINGS[3] = 1;			//TIME MODE DEFAULT: 0 - elapsed; 1 - remain*;
		UTILITY_SETTINGS[4] = 1;			//TEMPO RANGE DEFAULT: 0 - 6%; 1 - 10%*; 2 - 16%; 3 - WIDE;  
		UTILITY_SETTINGS[5] = 1;			//SLIP FLASHING: 0 - OFF; 1 - ON*;  
		UTILITY_SETTINGS[6] = 0;			//SWITCH REVERSE MODE: 0 - REVERSE*; 1 - SLIP REVERSE;  
		UTILITY_SETTINGS[7] = 0;			//COLOR WAVEFORM: 0 - BLUE*; 1 - RGB;  
		UTILITY_SETTINGS[8] = 3;			//LCD BRIGHTNESS:  0 - 1; 1 - 2; 2 - 3; 3 - 4*; 4 - 5;   
		UTILITY_SETTINGS[9] = 1;			//JOG INDICATOR:  0 - OFF; 1 - ON*;
		UTILITY_SETTINGS[10] = 2;			//JOG BRIGHTNESS:  0 - 1; 1 - 2; 2 - 3*; 3 - 4; 4 - 5; 	
		UTILITY_SETTINGS[11] = 0;			//BPM COLOR: 0 - WHITE*; 1 - ORANGE;  
		UTILITY_SETTINGS[12] = 8;			//OUTPUT LEVEL:  0 - 10; 1 - 20; 2 - 30; 3 - 40; 4 - 50; 5 - 60; 6 - 70; 7 - 80; 8 - 90*; 9 - 100; 
		need_rewrite_flash = 0;	
		}
		
	REMAIN_ENABLE = UTILITY_SETTINGS[3]%2; 
	tempo_range = UTILITY_SETTINGS[4]%4;
	
	HAL_Delay(500);
			
	BSP_LCD_DisplayOn();
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);		//PWM	JOG RING ON
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);		//LCD
	TIM12->CCR1 = JOG_BRIGHTNESS[UTILITY_SETTINGS[10]];		//PWM	JOG RING ON
	TIM14->CCR1 = 48+(52*UTILITY_SETTINGS[8]);		//LCD
	
	DrawLOGO();

	DEVICE_UID = *(unsigned long*)(0x1FF0F420);							//Read device ID
	
	BSP_LCD_SetFont(&Font15P);

	res = f_mount(&FAT, "0", 1);
	if (res!=FR_OK)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);	
		sprintf((char*)Buf, "%s", "Disc not mounted!");
		BSP_LCD_DisplayStringAt(10,1,Buf, LEFT_MODE);
		while(1)
			{}		
		}

	TOTAL_TRACKS = DATABASE_PARSER();
	
	if(TOTAL_TRACKS==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);	
		sprintf((char*)Buf, "%s", "Rekordbox database not found!");
		BSP_LCD_DisplayStringAt(10,30,Buf, LEFT_MODE);
		while(1)
			{}
		}
	else if(TOTAL_TRACKS==0xFFFF)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);	
		sprintf((char*)Buf, "%s", "Rekordbox database has more than 512 tracks.");
		BSP_LCD_DisplayStringAt(10,30,Buf, LEFT_MODE);
		sprintf((char*)Buf, "%s", "Download less than 512 tracks.");
		BSP_LCD_DisplayStringAt(10,50,Buf, LEFT_MODE);		
		while(1)
			{}	
		}
	else
		{
//		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
//		sprintf((char*)Buf, "Rekordbox database have a ""%03lu"" tracks", TOTAL_TRACKS);
//		BSP_LCD_DisplayStringAt(10,30,Buf, LEFT_MODE);		
		}	
		
		
	HAL_Delay(100);
	
	GET_SD_INFO();

	HAL_TIM_Base_Start_IT(&htim1);
		
	DAC_Init_and_Start(10*(UTILITY_SETTINGS[12]+1));											/////Start AUDIO PROCESSING
	
	HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin|LED_MENU_Pin, GPIO_PIN_RESET);				//power off unnecessary leds
	HAL_GPIO_WritePin(GPIOI, LED_INFO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, LED_MIDI_Pin|LED_USB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_RESET);	
	//////////////////////////////////////////////////////////////////////////////////
	
	BSP_LCD_Clear(LCD_COLOR_BLACK);	
	DrawStaticInformation();
	ShowQUANTIZE(1);	
	DrawTempoRange(tempo_range);			
	ShowTempo(10000);
	ShowBPM(originalBPM);
	ShowTrackNumber(track_play_now);
	ShowREMAIN(REMAIN_ENABLE);	
	if(UTILITY_SETTINGS[2]==8)
		{
		ShowACUE(2);	
		}
	else
		{
		ShowACUE(1);	
		}
	SwitchInformationLayer(WAVEFORM);		//Show browser
	RedrawWaveforms(0);
	DrawCueMarker(0);	
	DrawStaticWFM(CLEAR_WAVEFORM_ARRAY);
	DrawStaticWFM(MS_NOT_LOADED);	
	
	DrawREKORDBOX();

	uint8_t JJ;			//variable for internal calc
	uint8_t PART_CODE = 0; //part of code when run in main()	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {			
	RedrawWaveforms(play_adr/294);	
		
	if(end_adr_valid_data<128)
		{
		f_read(&file, PCM[end_adr_valid_data][0], 32768, &nbytes);
		//DrawCueMarker(1+((end_adr_valid_data*11145)/all_long));				////-----------------
		end_adr_valid_data++;	
		}
	else if((end_adr_valid_data<((play_adr>>13)+42)) && (filling_step==0 || filling_step==6))									//filling the buffer forward
		{
		if(filling_step==6)
			{
			f_lseek(&file, ((32768*end_adr_valid_data)+44));	
			filling_step = 0;	
			}
		f_read(&file, PCM[end_adr_valid_data&0x7F][0], 32768, &nbytes);
		//DrawCueMarker(1+((end_adr_valid_data*11145)/all_long));			////-----------------
		end_adr_valid_data++;
		if((end_adr_valid_data-start_adr_valid_data)>128)
			{
			start_adr_valid_data = end_adr_valid_data-128;	
			}
		}
	else if(((end_adr_valid_data>((play_adr>>13)+86) || ((end_adr_valid_data-start_adr_valid_data)<124)) && start_adr_valid_data>3) || (filling_step!=0 && filling_step!=6))					//filling the buffer back
		{
		if(filling_step==0 || filling_step==6)
			{
			if((end_adr_valid_data-start_adr_valid_data)>127)	
				{
				end_adr_valid_data = start_adr_valid_data+124;	
				}	
			start_adr_valid_data-= 4;	
			f_lseek(&file, ((32768*(start_adr_valid_data))+44));
			filling_step = 1;	
			}
		else if(filling_step==1)
			{
			f_read(&file, PCM[start_adr_valid_data&0x7F][0], 32768, &nbytes);
			filling_step = 2;	
			}
		else if(filling_step==2)
			{
			f_read(&file, PCM[(start_adr_valid_data+1)&0x7F][0], 32768, &nbytes);
			filling_step = 3;	
			}
		else if(filling_step==3)
			{
			f_read(&file, PCM[(start_adr_valid_data+2)&0x7F][0], 32768, &nbytes);
			filling_step = 4;	
			}
		else if(filling_step==4)
			{
			f_read(&file, PCM[(start_adr_valid_data+3)&0x7F][0], 32768, &nbytes);
			filling_step = 5;	
			}
		else if(filling_step==5)
			{
			//DrawCueMarker(1+((start_adr_valid_data*11145)/all_long));											////-----------------
			filling_step = 6;		
			}
		}	



	if(PART_CODE==0)
		{
		TOUCH_SCREEN_HANDLER();
		PART_CODE = 1;	
		}
	else if(PART_CODE==1)
		{	
		if(track_need_load!=0 && TOTAL_TRACKS_IN_CURRENT_PLAYLIST!=0)
			{
			if(UTILITY_SETTINGS[1] && end_of_track==0 && play_enable)						//lock load	
				{	
				//message about load lock		
				}
			else
				{	
				if(track_need_load==1)				//load next track
					{				
					if(track_play_now==TOTAL_TRACKS)
						{
						track_play_now = 1;	
						}
					else
						{
						track_play_now++;	
						}	
					}
				else if(track_need_load==2) 	//load previous track
					{
					if(track_play_now==1 || track_play_now==0)
						{
						track_play_now = TOTAL_TRACKS;	
						}
					else
						{
						track_play_now--;	
						}	
					}
				PREPARE_LOAD_TRACK(track_play_now, track_play_now);			
				}	
			track_need_load = 0;	
			}
			
		if(tempo_range_need_update)
			{	
			DrawTempoRange(tempo_range);
			tempo_need_update = 1;	
			tempo_range_need_update = 0;	
			}
			
		if(tempo_need_update>0)
			{
			if(tempo_need_update==1)
				{
				ShowTempo(potenciometer_tempo);	
				}	
			if(originalBPM!=0xFFFF)
				{
				ShowBPM(((originalBPM+5)*potenciometer_tempo)/100000);	
				}
			tempo_need_update = 0;
			}

		if(time_mode_need_update)			
			{
			forcibly_redraw = 1;	
			ShowREMAIN(REMAIN_ENABLE);
			if(REMAIN_ENABLE)
				{
				DrawStaticWFM(REDRAW_IN_REMAIN_MODE);
				}
			else
				{
				DrawStaticWFM(REDRAW_IN_NREMAIN_MODE);	
				}
			time_mode_need_update = 0;		
			}
		else if(need_DSW>0)
			{
			if(need_DSW==1)
				{
				DrawStaticWFM(REDRAW_IN_NREMAIN_MODE);				
				}
			else
				{
				DrawStaticWFM(REDRAW_IN_REMAIN_MODE);					
				}
			if(UTILITY_SETTINGS[9])					//jog indicator ON
				{	
				if(TIM12->CCR1==0)	
					{
					TIM12->CCR1 = JOG_BRIGHTNESS[UTILITY_SETTINGS[10]];		
					}	
				else
					{
					TIM12->CCR1 = 0;	
					}
				}	
			need_DSW = 0;	
			}
		else if(quantize_mode_need_update)
			{
			if(QUANTIZE==0)
				{
				ShowQUANTIZE(0);	
				}
			else
				{
				if(track_play_now==0)
					{
					ShowQUANTIZE(1);	
					}
				else
					{
					ShowQUANTIZE(2);
					}					
				}	
			quantize_mode_need_update = 0;	
			}		
		PART_CODE = 2;	
		}		
	else if(PART_CODE==2)
		{
		if(HAL_GPIO_ReadPin(KEY_INFO_GPIO_Port, KEY_INFO_Pin)==0 & KEY_INFO_pressed==0)					//INFO BUTTON
			{
			if(dSHOW==TAG_LIST)
				{
				TAG_LIST_INFO_enable = 1;	
				SwitchInformationLayer(TAG_LIST_INFO);
				}	
			else if(dSHOW==TAG_LIST_INFO)
				{
				TAG_LIST_INFO_enable = 0;	
				SwitchInformationLayer(TAG_LIST);
				}
			else if(dSHOW==BROWSER && BROWSE_LEVEL==0)
				{
				BROWSER_INFO_enable = 1;	
				SwitchInformationLayer(BROWSER_INFO);
				}		
			else if(dSHOW==BROWSER_INFO && BROWSE_LEVEL==0)
				{
				BROWSER_INFO_enable = 0;		
				SwitchInformationLayer(BROWSER);
				}		
			KEY_INFO_pressed = 1;	
			}
		else if(HAL_GPIO_ReadPin(KEY_INFO_GPIO_Port, KEY_INFO_Pin)==1 & KEY_INFO_pressed==1)
			{
			KEY_INFO_pressed = 0;		
			}	
			
			
		if(HAL_GPIO_ReadPin(GPIOG, KEY_BROWSE_Pin)==0 & KEY_BROWSE_pressed==0)					//BROWSER-WAVEFORM BUTTON
			{
			if(dSHOW != BROWSER && dSHOW != BROWSER_INFO)
				{
				if(BROWSE_LEVEL==0 && BROWSER_INFO_enable==0)
					{
					SwitchInformationLayer(BROWSER);	
					}
				else
					{
					SwitchInformationLayer(BROWSER_INFO);	
					}
				}
			else
				{
				SwitchInformationLayer(WAVEFORM);	
				}
			KEY_BROWSE_pressed = 1;	
			}
		else if(HAL_GPIO_ReadPin(GPIOG, KEY_BROWSE_Pin)==1 & KEY_BROWSE_pressed==1)
			{
			KEY_BROWSE_pressed = 0;		
			}
		else if(HAL_GPIO_ReadPin(GPIOF, KEY_SD_Pin)==0 & KEY_SD_pressed==0)					//SD BUTTON
			{	
			if(dSHOW != BROWSER && dSHOW != BROWSER_INFO)
				{
				if(BROWSE_LEVEL==0 && BROWSER_INFO_enable==0)
					{
					SwitchInformationLayer(BROWSER);	
					}
				else
					{
					SwitchInformationLayer(BROWSER_INFO);	
					}
				}	
			KEY_SD_pressed = 1;	
			}
		else if(HAL_GPIO_ReadPin(GPIOF, KEY_SD_Pin)==1 & KEY_SD_pressed==1)	
			{
			KEY_SD_pressed = 0;	
			}
		else if(HAL_GPIO_ReadPin(GPIOF, KEY_MIDI_Pin)==0 & KEY_MIDI_pressed==0)					//MIDI BUTTON
			{	
			/////////////////////////////
			CUE_OPERATION = MEMORY_NEED_NEXT_SET;	
				
			HAL_GPIO_WritePin(GPIOF, LED_MIDI_Pin, GPIO_PIN_SET);			
			KEY_MIDI_pressed = 1;		
			}
		else if(HAL_GPIO_ReadPin(GPIOF, KEY_MIDI_Pin)==1 & KEY_MIDI_pressed==1)	
			{
			HAL_GPIO_WritePin(GPIOF, LED_MIDI_Pin, GPIO_PIN_RESET);				
			KEY_MIDI_pressed = 0;	
			}			
		else if(HAL_GPIO_ReadPin(GPIOF, KEY_USB_Pin)==0 & KEY_USB_pressed==0)					//USB BUTTON
			{
			//CALL_CUE();	
			////////////////////////
				
			HAL_GPIO_WritePin(GPIOF, LED_USB_Pin, GPIO_PIN_SET);			
			KEY_USB_pressed = 1;		
			}
		else if(HAL_GPIO_ReadPin(GPIOF, KEY_USB_Pin)==1 & KEY_USB_pressed==1)	
			{
			//offset_adress = 0;																///	   temporary operation
			HAL_GPIO_WritePin(GPIOF, LED_USB_Pin, GPIO_PIN_RESET);				
			KEY_USB_pressed = 0;	
			}	
					
			
		if(HAL_GPIO_ReadPin(GPIOG, KEY_TAG_TRACK_Pin)==0 & KEY_TAG_TRACK_pressed==0)					//TAG TRACK/REMOVE BUTTON
			{
			if((dSHOW==BROWSER || dSHOW==BROWSER_INFO) && BROWSE_LEVEL==0)
				{
				if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]&0x2)==0)					//////////////when track is not in tag list
					{
					if(TOTAL_TRACKS_IN_TAG_LIST<100)
						{
						playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]|=0x02;		//////////////////write add taglist mark
						TAG_LIST_BASE[TOTAL_TRACKS_IN_TAG_LIST] = TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1];
						TOTAL_TRACKS_IN_TAG_LIST++;
						}
					}
				else																													///////////////////////Delete Track from TAG LIST
					{
					playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]&=0xFD;		//write delete taglist mark	
						s = 0;
					while(TAG_LIST_BASE[s]!=(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]) && s<TOTAL_TRACKS_IN_TAG_LIST)
						{
						s++;		
						}
					TOTAL_TRACKS_IN_TAG_LIST--;	
					while(s<TOTAL_TRACKS_IN_TAG_LIST)
						{
						TAG_LIST_BASE[s] = TAG_LIST_BASE[s+1];	
						s++;	
						}
					if(TOTAL_TRACKS_IN_TAG_LIST<(TCurrentCursorPosition + TCurrentTrackPosition))
						{
						if(TOTAL_TRACKS_IN_TAG_LIST>7)
							{
							TCurrentTrackPosition-=1;		
							}
						else
							{
							if(TCurrentCursorPosition>0)
								{
								TCurrentCursorPosition-=1;
								}
							}		
						}	
					}
				SwitchInformationLayer(dSHOW);
				}
			else if((dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO) && TOTAL_TRACKS_IN_TAG_LIST>0)
				{
				playlist[TAG_LIST_BASE[TCurrentCursorPosition + TCurrentTrackPosition-1]-1][54]&=0xFD;		//delete taglist mark	
				s = TCurrentCursorPosition + TCurrentTrackPosition-1;
				TOTAL_TRACKS_IN_TAG_LIST--;
					
					
				while(s<TOTAL_TRACKS_IN_TAG_LIST)
					{
					TAG_LIST_BASE[s] = TAG_LIST_BASE[s+1];	
					s++;	
					}
					
				if(TOTAL_TRACKS_IN_TAG_LIST<(TCurrentCursorPosition + TCurrentTrackPosition))
					{
					if(TOTAL_TRACKS_IN_TAG_LIST>7)
						{
						TCurrentTrackPosition-=1;		
						}
					else
						{
						if(TCurrentCursorPosition>0)
							{
							TCurrentCursorPosition-=1;
							}
						}		
					}
				SwitchInformationLayer(dSHOW);			
				}
			KEY_TAG_TRACK_pressed = 1;	
			}
		else if(HAL_GPIO_ReadPin(GPIOG, KEY_TAG_TRACK_Pin)==1 & KEY_TAG_TRACK_pressed==1)	
			{
			KEY_TAG_TRACK_pressed = 0;	
			}
				
			
		if(HAL_GPIO_ReadPin(GPIOC, KEY_TAG_LIST_Pin)==0 & KEY_TAG_LIST_pressed==0)					//TAG LIST BUTTON
			{
			if(dSHOW != TAG_LIST && dSHOW != TAG_LIST_INFO)
				{
				if(TAG_LIST_INFO_enable)
					{
					SwitchInformationLayer(TAG_LIST_INFO);
					}
				else
					{
					SwitchInformationLayer(TAG_LIST);	
					}	
				}
			else if(dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO)
				{
				SwitchInformationLayer(WAVEFORM);
				}
			KEY_TAG_LIST_pressed = 1;	
			}
		else if(HAL_GPIO_ReadPin(GPIOC, KEY_TAG_LIST_Pin)==1 & KEY_TAG_LIST_pressed==1)	
			{
			KEY_TAG_LIST_pressed = 0;	
			}		
		

		if(HAL_GPIO_ReadPin(KEY_MENU_GPIO_Port, KEY_MENU_Pin)==0 & KEY_MENU_pressed==0)					//MENU BUTTON
			{
			if(dSHOW != UTILITY)
				{
				if(countUTILITY==6)
					{
					edit_parameter = 0;	
					SwitchInformationLayer(UTILITY);
					KEY_MENU_pressed = 1;	
					}	
				}	
				else
				{	
				SwitchInformationLayer(WAVEFORM);
				KEY_MENU_pressed = 1;		
				}				
			}
		else if(HAL_GPIO_ReadPin(KEY_MENU_GPIO_Port, KEY_MENU_Pin)==1 & KEY_MENU_pressed==1)	
			{	
			KEY_MENU_pressed = 0;	
			}	
			
		
		if(need_up)	
			{
			if(dSHOW==BROWSER || dSHOW==BROWSER_INFO)
				{
				if(BROWSE_LEVEL==0)
					{
					NAVIGATOR(BROWSER0_UP);	
					}
				else if(BROWSE_LEVEL==1)
					{
					NAVIGATOR(BROWSER1_UP);		
					}
				else if(BROWSE_LEVEL==2)
					{
					NAVIGATOR(BROWSER2_UP);	
					}	
				}
			else if(dSHOW==WAVEFORM)
				{
				if(DynamicWaveformZOOM>1)
					{
					DynamicWaveformZOOM = DynamicWaveformZOOM/2;
					if(track_play_now!=0)
						{
						forcibly_redraw = 1;
						}			
					}				
				}
			else if(dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO)
				{
				NAVIGATOR(TAGLIST_UP);	
				}
			else if(dSHOW==UTILITY)
				{
				if(edit_parameter==0)
					{
					NAVIGATOR(UTILITY_UP);
					}
				else
					{
					if(UTILITY_SETTINGS[UCurrentCursorPosition+CurrentUPosition-1]<UTILITY_SETTINGS_MAX[UCurrentCursorPosition+CurrentUPosition-1])	
						{
						UTILITY_SETTINGS[UCurrentCursorPosition+CurrentUPosition-1]++;	
						int_U_REDRAW_ONE_LINE();	
						int_reload_parameter_realtime();					
						}					
					}				
				}
			need_up = 0;	
			}
		else if(need_down)
			{	
			if(dSHOW==BROWSER || dSHOW==BROWSER_INFO)
				{
				if(BROWSE_LEVEL==0)
					{
					NAVIGATOR(BROWSER0_DOWN);	
					}
				else if(BROWSE_LEVEL==1)
					{
					NAVIGATOR(BROWSER1_DOWN);			
					}
				else if(BROWSE_LEVEL==2)
					{
					NAVIGATOR(BROWSER2_DOWN);		
					}			
				}
			else if(dSHOW==WAVEFORM)
				{				
				if(DynamicWaveformZOOM<16)
					{
					DynamicWaveformZOOM = DynamicWaveformZOOM*2;
					if(track_play_now!=0)
						{	
						forcibly_redraw = 1;
						}
					}
				}
			else if(dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO)
				{
				NAVIGATOR(TAGLIST_DOWN);	
				}
			else if(dSHOW==UTILITY)
				{
				if(edit_parameter==0)
					{
					NAVIGATOR(UTILITY_DOWN);
					}	
				else
					{					
					if(UTILITY_SETTINGS[UCurrentCursorPosition+CurrentUPosition-1]>0)
						{
						UTILITY_SETTINGS[UCurrentCursorPosition+CurrentUPosition-1]--;	
						int_U_REDRAW_ONE_LINE();		
						int_reload_parameter_realtime();	
						}
					}	
				}
			need_down = 0;	
			}
		
			
		if(HAL_GPIO_ReadPin(GPIOI, KEY_BACK_Pin)==0 & KEY_BACK_pressed==0)					//BACK BUTTON
			{
			if((dSHOW==BROWSER || dSHOW==BROWSER_INFO) && BROWSE_LEVEL<3)
				{
				PREVIOUS_BROWSE_LEVEL =	BROWSE_LEVEL; 	
				BROWSE_LEVEL+=1;
				SwitchInformationLayer(BROWSER_NAVI);		
				}
			KEY_BACK_pressed = 1;		
			}
		else if(HAL_GPIO_ReadPin(GPIOI, KEY_BACK_Pin)==1 & KEY_BACK_pressed==1)	
			{	
			KEY_BACK_pressed = 0;	
			}		
		else if(HAL_GPIO_ReadPin(GPIOI, ENC_BUTTON_Pin)==0 && LOAD_pressed==0) ///////////////////LOAD TRACK
			{
			if((BROWSE_LEVEL==0 &&(dSHOW==BROWSER || dSHOW==BROWSER_INFO)) || ((dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO) && TOTAL_TRACKS_IN_TAG_LIST>0)) //LOAD TRACK from taglist or playlist
				{	
				if(UTILITY_SETTINGS[1] && end_of_track==0 && play_enable)						//lock load	
					{	
					//message about load lock		
					}	
				else	
					{
					ENCODER_LED_BLINK = 0;
					load_animation_enable = 1;	
					if(dSHOW==BROWSER || dSHOW==BROWSER_INFO)
						{
						track_play_now = TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition + BCurrentTrackPosition - 1];	
						TRACK_PLAY_IN_CURRENT_PLAYLIST = B0CurrentCursorPosition + BCurrentTrackPosition;
						}
					else if(dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO)
						{
						track_play_now = TAG_LIST_BASE[TCurrentCursorPosition + TCurrentTrackPosition-1];
						TRACK_PLAY_IN_CURRENT_PLAYLIST = TCurrentCursorPosition + TCurrentTrackPosition;
						}
					PREPARE_LOAD_TRACK(track_play_now, TRACK_PLAY_IN_CURRENT_PLAYLIST);		
					}				
				}
			else if((dSHOW==BROWSER || dSHOW==BROWSER_INFO) && BROWSE_LEVEL>0)
				{
				if(BROWSE_LEVEL==2 && (B2CurrentCursorPosition==0 || B2CurrentCursorPosition==1 || B2CurrentCursorPosition==3))	
					{
					//////////////////////////////////////	
					}
				else
					{
					PREVIOUS_BROWSE_LEVEL =	BROWSE_LEVEL; 
					if(BROWSE_LEVEL==1)			//enter to playlist
						{
						TOTAL_TRACKS_IN_CURRENT_PLAYLIST = TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition] - TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1];
						B0CurrentCursorPosition = 0;
						BCurrentTrackPosition = 1;	
						}
					BROWSE_LEVEL-=1;
					SwitchInformationLayer(BROWSER_NAVI);
					}					
				}
			else if(dSHOW==UTILITY)
				{
				if(edit_parameter==0 && ((UCurrentCursorPosition + CurrentUPosition)<14))
					{
					need_rewrite_flash = 1;	
					edit_parameter = 1;	
					SwitchInformationLayer(UTILITY);		
					}
				else if(edit_parameter)
					{
					edit_parameter = 0;		
					SwitchInformationLayer(UTILITY);
					int_reload_parameter();	//apply new parameters	
					}
				}
			LOAD_pressed = 1;	
			}	
		//else if(HAL_GPIO_ReadPin(GPIOI, ENC_BUTTON_Pin)==1 && (dSHOW==BROWSER || dSHOW==BROWSER_INFO || ((dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO) && TOTAL_TRACKS_IN_TAG_LIST>0)) && LOAD_pressed==1)
		else if(HAL_GPIO_ReadPin(GPIOI, ENC_BUTTON_Pin)==1 && LOAD_pressed==1)
			{
			LOAD_pressed = 0;		
			}
		PART_CODE = 0;	
		}		
		
																										
		
	if(loop_active && CUE_ADR<LOOP_OUT && (play_adr/294)>=LOOP_OUT)					//return to cue for loop mode
		{	
		CALL_CUE();
		//offset_adress = 128-mem_offset_adress;	
		ftp = 1;	
		}
	else if(ftp!=0 && ftp<15)
		{
		ftp++;	
		}
	else if(ftp==15)	
		{
		offset_adress = 0;
		ftp = 0;		
		}
			
		
	if(CUE_OPERATION==CUE_NEED_SET)
		{
		if(QUANTIZE && dSHOW==WAVEFORM)				//add calculate bars in background process
			{
			if(((play_adr/294)>(BEATGRID[bars-1]+((BEATGRID[bars] - BEATGRID[bars-1])/2))) || bars==0)	
				{
				SET_CUE(BEATGRID[bars]);	
				}
			else
				{
				SET_CUE(BEATGRID[bars-1]);		
				}				
			}
		else
			{
			SET_CUE(play_adr/294);	
			}
		CUE_OPERATION = 0;	
		}
	else if(CUE_OPERATION==CUE_NEED_CALL)
		{
		CALL_CUE();
		CUE_OPERATION = 0;			
		}
	else if(CUE_OPERATION==MEMORY_NEED_NEXT_SET)
		{
		if(number_of_memory_cue_points>0)
			{
			JJ=0;
			while(MEMORY_adr[0][JJ]<=(play_adr/294) && (JJ<number_of_memory_cue_points-1))
				{
				JJ++;	
				}
			if((play_adr/294)<MEMORY_adr[0][JJ])
				{
				if(loop_active)				//deactivate loop
					{
					loop_active = 0;
					LOOP_OUT = 0;			
					}
				SET_MEMORY_CUE_1(MEMORY_adr[0][JJ]);
				CUE_OPERATION = MEMORY_NEED_SET_PART2;	
				}
			else
				{
				CUE_OPERATION = 0;	
				}
			}
		else
			{
			CUE_OPERATION = 0;	
			}
		}	
	else if(CUE_OPERATION==MEMORY_NEED_PREVIOUS_SET)
		{
		if(number_of_memory_cue_points>0)
			{
			JJ = number_of_memory_cue_points-1;
			while(MEMORY_adr[0][JJ]>=(play_adr/294) && (JJ>0))
				{
				JJ--;	
				}
			if((play_adr/294)>MEMORY_adr[0][JJ])
				{
				if(loop_active)				//deactivate loop
					{
					loop_active = 0;
					LOOP_OUT = 0;			
					}	
				SET_MEMORY_CUE_1(MEMORY_adr[0][JJ]);
				CUE_OPERATION = MEMORY_NEED_SET_PART2;	
				}
			else
				{
				CUE_OPERATION = 0;	
				}
			}
		else
			{
			CUE_OPERATION = 0;	
			}			
		}	
	else if(CUE_OPERATION==MEMORY_NEED_SET_PART2 && ((end_adr_valid_data-start_adr_valid_data)>64))	
		{
		SET_MEMORY_CUE_2();
		offset_adress = 0;		
		CUE_OPERATION = 0;
		}
		
		
		
		
		
		
		
	if(animation_en!=0)
		{			
		animation_step = 2*(HAL_GetTick() - animation_time); //0,24 sec for animation	
		if(animation_step<480 || animation_finish)				
			{	
			CURRENT_LAY = ActiveLayer;	
			if(ActiveLayer !=1)
				{
				BSP_LCD_SelectLayer(1);
				}	
			if(animation_step>479 && animation_finish)
				{
				animation_step = 479; 	
				animation_finish = 0;	
				}
			if(previous_animation_step != animation_step)	
				{
				if(animation_en==1)			//forward animation
					{					
					if(animation_step<271)
						{
						BSP_LCD_SetTextColor(LCD_COLOR_PAPER_TRANSP);					//Draw paper rectangle transparent
						BSP_LCD_FillRect(270-animation_step, 18, animation_step-previous_animation_step, 152);	
						}
					BSP_LCD_SetTextColor(0x00000000);					//transparent
					BSP_LCD_FillRect(479-animation_step, 18, animation_step-previous_animation_step+1, 152);			
					if(animation_step>60 && info_animation_enable)
						{
						BSP_LCD_SetTextColor(LCD_COLOR_PAPER_TRANSP);					//Draw paper rectangle transparent	
						BSP_LCD_FillRect(479-((animation_step-61)/2), 18, (animation_step-previous_animation_step+1)/2, 152);	
						}
					previous_animation_step = animation_step;	
					}	
				else if(animation_en==2  && animation_step>previous_animation_step)		//back/reverse animation
					{
					BSP_LCD_SetTextColor(LCD_COLOR_PAPER_TRANSP);					//Draw paper rectangle transparent
					BSP_LCD_FillRect(previous_animation_step, 18, animation_step-previous_animation_step+1, 152);
					if(animation_step>210)
						{
						BSP_LCD_SetTextColor(0x00000000);					//transparent	
						if(previous_animation_step<211)
							{
							BSP_LCD_FillRect(0, 18, animation_step-210, 152);		
							}
						else
							{
							BSP_LCD_FillRect(previous_animation_step-211, 18, animation_step-previous_animation_step+1, 152);	
							}
						}
					if(animation_step<421 && info_animation_enable)
						{
						BSP_LCD_SetTextColor(0x00000000);
						if(previous_animation_step==0)
							{
							BSP_LCD_FillRect(270, 18, (animation_step+4)/2, 152);		
							}
						else
							{
							BSP_LCD_FillRect(270+(previous_animation_step/2), 18, (animation_step-previous_animation_step+1)/2, 152);		
							}
						}
					previous_animation_step = animation_step;		
					}
				}	
			BSP_LCD_SelectLayer(CURRENT_LAY);	
			}
		else
			{
			animation_en = 0;
			previous_animation_step = 0;	
			animation_step = 0;	
			animation_time = 0;	
			animation_finish = 1;	
			SwitchInformationLayer(BR_NAVI_END);	
			}
		}
		
		
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 135;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 3;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 120;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLI2SDivQ = 4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void DrawStaticInformation(void)
	{
	BSP_LCD_SetFont(&FontBMP);
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);		
	sprintf((char*)Buf, "%s", "67");						//PLAYER
	BSP_LCD_DisplayStringAt(4,186,Buf, TRANSPARENT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	sprintf((char*)Buf, "%s", "/0");						//TEMPO
	BSP_LCD_DisplayStringAt(325,185,Buf, TRANSPARENT_MODE);
	if(UTILITY_SETTINGS[0]==0)			//single	
		{
		sprintf((char*)Buf, "%s", "CD");						//SINGLE	
		}
	else
		{
		sprintf((char*)Buf, "%s", "23");						//TRACK	
		}
	BSP_LCD_DisplayStringAt(47,183,Buf, TRANSPARENT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font23D);
	sprintf((char*)Buf, "%s", "m");			//M:
	BSP_LCD_DisplayStringAt(176,194,Buf, TRANSPARENT_MODE);	
	sprintf((char*)Buf, "%s", "s");			//S
	BSP_LCD_DisplayStringAt(230,194,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", ".");			//dot
	BSP_LCD_DisplayStringAt(284,194,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "f");			//F
	BSP_LCD_DisplayStringAt(309,194,Buf, TRANSPARENT_MODE);
		
	BSP_LCD_SetFont(&Font18D);	
	sprintf((char*)Buf, "%s", "%");			//PITCH procent symbol
	BSP_LCD_DisplayStringAt(400,199,Buf, TRANSPARENT_MODE);

	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	sprintf((char*)Buf, "%s", "#");			//PLAYER NUBMER 1
	BSP_LCD_DisplayStringAt(15,201,Buf, TRANSPARENT_MODE);
	
	BSP_LCD_DrawLine(3, 221, 36, 221);					//PLAYER shield
	BSP_LCD_DrawLine(1, 184, 1, 219);
	BSP_LCD_DrawPixel(2, 183, 0xFF808080);
	BSP_LCD_DrawPixel(2, 220, 0xFF808080);
	BSP_LCD_DrawLine(3, 182, 36, 182);
	BSP_LCD_DrawLine(38, 184, 38, 219);
	BSP_LCD_DrawPixel(37, 183, 0xFF808080);
	BSP_LCD_DrawPixel(37, 220, 0xFF808080);	
	}

///////////////////////////	
//show A.CUE on display
//	0 - none	
//	1 - red
//	2 - white	
//	
void ShowACUE(uint8_t acue)
	{	
	BSP_LCD_SetFont(&FontBMP);	
	if(acue==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);	
		}	
	else if(acue==2)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);	
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}
	sprintf((char*)Buf, "%s", "%&");						//A.CUE
	BSP_LCD_DisplayStringAt(93,206,Buf, TRANSPARENT_MODE);
	}
	
///////////////////////////	
//show REMAIN on display
//
void ShowREMAIN(uint8_t remain)
	{
	BSP_LCD_SetFont(&FontBMP);
	if(remain==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);	
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}
	sprintf((char*)Buf, "%s", "-.");						//REMAIN	
	BSP_LCD_DisplayStringAt(93,196,Buf, TRANSPARENT_MODE);
	}
	
	
	
///////////////////////////	
//show QUANTIZE on display
//	0 - none 1 - gray 2 - red	
void ShowQUANTIZE(uint8_t color)
	{
	BSP_LCD_SetFont(&FontBMP);
	if(color==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);	
		}
	else if(color==2)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}		
	sprintf((char*)Buf, "%s", "AB");						//QUANTIZE	
	BSP_LCD_DisplayStringAt(217, 185, Buf, TRANSPARENT_MODE);	
	}
	
	

//show bpm and bpm shield
//
// input bpm*10
//	
void ShowBPM(uint16_t bpm)
	{		
	uint32_t clr;		
	if(UTILITY_SETTINGS[11]==0)
		{
		clr	= LCD_COLOR_WHITE;
		}				
	else
		{
		clr	= CUE_COLOR;	
		}		
	
	if(bpm != 0xFFFF)
		{
		bpm = bpm%10000; 	
		BSP_LCD_SetTextColor(clr);
		BSP_LCD_SetFont(&Font20D);
		sprintf((char*)Buf, "%3lu", bpm/10);			//BPM count
		BSP_LCD_DisplayStringAt(418,187,Buf, LEFT_MODE);
		BSP_LCD_DrawPixel(466, 205, clr);	//
		BSP_LCD_DrawPixel(466, 206, clr);	//	DOT
		BSP_LCD_DrawPixel(465, 205, clr);	//
		BSP_LCD_DrawPixel(465, 206, clr);	//
			
		BSP_LCD_SetFont(&Font11D);
		sprintf((char*)Buf, "%01lu", bpm%10);			//BPM count
		BSP_LCD_DisplayStringAt(469,196,Buf, LEFT_MODE);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(418, 187, 58, 20);	
		BSP_LCD_SetTextColor(clr);		
		}

	BSP_LCD_SetFont(&FontBMP);
	sprintf((char*)Buf, "%s", "1");						//BPM
	BSP_LCD_DisplayStringAt(454,212,Buf, TRANSPARENT_MODE);
		
	BSP_LCD_DrawLine(416, 219, 476, 219);					//BPM shield
	BSP_LCD_DrawLine(478, 217, 478, 184);
	BSP_LCD_DrawPixel(477, 218, clr);
	BSP_LCD_DrawPixel(477, 183, clr);
	BSP_LCD_DrawLine(416, 182, 476, 182);
	BSP_LCD_DrawLine(414, 217, 414, 184);
	BSP_LCD_DrawPixel(415, 218, clr);
	BSP_LCD_DrawPixel(415, 183, clr);		
	return;	
	}

//////////////////////////////////////////////////	
//Draw time position bar 0...399
//
//	DRAW_NEW_STATIC_WAVEFORM		400
//	CLEAR_WAVEFORM_ARRAY				401
//	MS_NOT_LOADED								402
//	REDRAW_IN_NREMAIN_MODE				
//	REDRAW_IN_REMAIN_MODE						
//	MS_ERROR										410
//
void DrawStaticWFM(uint16_t Tpos)
	{
	uint16_t i;
	if(Tpos<400)
		{	
		if(previous_position_bar != Tpos)			
			{
			ForceDrawVLine(previous_position_bar+40, 230, 36, LCD_COLOR_BLACK);
			ForceDrawVLine(previous_position_bar+41, 230, 36, LCD_COLOR_BLACK);					
			ForceDrawVLine(previous_position_bar+40, 256-(WFORMSTATIC[previous_position_bar]&0x1F), (WFORMSTATIC[previous_position_bar]&0x1F)+1, 
				WS_COLOR_MAP[UTILITY_SETTINGS[7]][WFORMSTATIC[previous_position_bar]>>7][(WFORMSTATIC[previous_position_bar]>>2)&0x07]);	
			ForceDrawVLine(previous_position_bar+41, 256-(WFORMSTATIC[previous_position_bar+1]&0x1F), (WFORMSTATIC[previous_position_bar+1]&0x1F)+1, 
				WS_COLOR_MAP[UTILITY_SETTINGS[7]][WFORMSTATIC[previous_position_bar+1]>>7][(WFORMSTATIC[previous_position_bar+1]>>2)&0x07]);

			if(previous_position_bar>Tpos)						//___<<||___     moving
				{				
				i = previous_position_bar - Tpos;
				if(REMAIN_ENABLE)
					{	
					while(i>0)
						{
						if((Tpos+i)%2==0 && (Tpos+i<398))
							{
							ForceDrawVLine(Tpos+i+42, 259, 4, LCD_COLOR_WHITE);	
							}
						else
							{
							ForceDrawVLine(Tpos+i+42, 259, 4, PBAR_COLOR_1);	
							}
							i--;	
						}
					}
				else
					{
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
					BSP_LCD_FillRect(Tpos+41, 260, (previous_position_bar-Tpos+2), 2);			
					while(i>0)
						{
						if(((Tpos+i)%2==0) && (Tpos+i<398))
							{
							BSP_LCD_DrawPixel(Tpos+i+42, 259, PBAR_COLOR_2);	
							BSP_LCD_DrawPixel(Tpos+i+42, 262, PBAR_COLOR_2);		
							}
						else if(((Tpos+i)%2==1) && (Tpos+i<398))
							{
							BSP_LCD_DrawPixel(Tpos+i+42, 259, PBAR_COLOR_3);	
							BSP_LCD_DrawPixel(Tpos+i+42, 262, PBAR_COLOR_3);		
							}
						i--;	
						}
					ForceDrawVLine(439, 259, 4, PBAR_COLOR_2);						//gray scroll vertical line in end					
					}
				}
			else if(previous_position_bar<Tpos)																			//___||>>___     moving
				{
				i = Tpos - previous_position_bar+1;	
				if(REMAIN_ENABLE)																		
					{
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
					BSP_LCD_FillRect(previous_position_bar+41, 260, (Tpos - previous_position_bar+2), 2);						
					ForceDrawVLine(40, 259, 4, PBAR_COLOR_2);						//gray scroll vertical line in start	
					while(i>0)
						{
						if((previous_position_bar+i)>1)	
							{
							if((previous_position_bar+i)%2==0)
								{
								BSP_LCD_DrawPixel(38+previous_position_bar+i, 259, PBAR_COLOR_2);	
								BSP_LCD_DrawPixel(38+previous_position_bar+i, 262, PBAR_COLOR_2);	
								}
							else
								{
								BSP_LCD_DrawPixel(38+previous_position_bar+i, 259, PBAR_COLOR_3);	
								BSP_LCD_DrawPixel(38+previous_position_bar+i, 262, PBAR_COLOR_3);		
								}
							}
						i--;					
						}
					}
				else				
					{
					while(i>0)
						{
						if((previous_position_bar+i)>1)	
							{
							if((previous_position_bar+i)%2==0)
								{
								ForceDrawVLine(38+previous_position_bar+i, 259, 4, LCD_COLOR_WHITE);		
								}
							else
								{
								ForceDrawVLine(38+previous_position_bar+i, 259, 4, PBAR_COLOR_1);		
								}
							}
						i--;					
						}
					}
				}			
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_FillRect(Tpos+39, 259, 4, 4);	
			
			if(RED_VERTICAL_LINE)						//detecting touch on sensor or touch on jog 
				{
				ForceDrawVLine(Tpos+40, 230, 36, LCD_COLOR_RED);		//red vertical scroll
				ForceDrawVLine(Tpos+41, 230, 36, LCD_COLOR_RED);		
				}
			else
				{
				ForceDrawVLine(Tpos+40, 230, 36, LCD_COLOR_WHITE);		//white vertical scroll
				ForceDrawVLine(Tpos+41, 230, 36, LCD_COLOR_WHITE);		
				}
			previous_position_bar = Tpos;	
			}
		else if(forcibly_redraw==1)
			{
			if(RED_VERTICAL_LINE)						//detecting touch on sensor or touch on jog 
				{
				ForceDrawVLine(Tpos+40, 230, 36, LCD_COLOR_RED);		//red vertical scroll
				ForceDrawVLine(Tpos+41, 230, 36, LCD_COLOR_RED);			
				}
			else
				{
				ForceDrawVLine(Tpos+40, 230, 36, LCD_COLOR_WHITE);		//white vertical scroll
				ForceDrawVLine(Tpos+41, 230, 36, LCD_COLOR_WHITE);		
				}
			}
		return;	
		}
	else if(Tpos==REDRAW_IN_NREMAIN_MODE)
		{
		if(previous_position_bar<397)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			BSP_LCD_FillRect(previous_position_bar+42, 260, 397-previous_position_bar, 2);
			}				
		for(i=0;i<400;i++)						
			{			
			if(i<previous_position_bar-1)
				{
				if(i%2==0)							//drawing ||||||||||||||||||
					{
					ForceDrawVLine(i+40, 259, 4, LCD_COLOR_WHITE);		
					}
				else
					{
					ForceDrawVLine(i+40, 259, 4, PBAR_COLOR_1);		
					}
				}	
			else if(i>previous_position_bar+2)
				{
				if(i%2==0)						//drawing :::::::::::::::
					{
					BSP_LCD_DrawPixel(40+i, 259, PBAR_COLOR_2);	
					BSP_LCD_DrawPixel(40+i, 262, PBAR_COLOR_2);	
					}
				else
					{
					BSP_LCD_DrawPixel(40+i, 259, PBAR_COLOR_3);	
					BSP_LCD_DrawPixel(40+i, 262, PBAR_COLOR_3);		
					}	
				}
			}
		if(previous_position_bar<397)
			{
			ForceDrawVLine(439, 259, 4, PBAR_COLOR_2);						//gray scroll vertical line in end	
			}	
		DRAWN_IN_REMAIN = 0;	
		return;	
		}
	else if(Tpos==REDRAW_IN_REMAIN_MODE)
		{
		if(previous_position_bar>1)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			BSP_LCD_FillRect(41, 260, previous_position_bar-1, 2);
			}
		for(i=0;i<400;i++)						
			{	
			if(i<previous_position_bar-1)
				{
				if(i%2==0)						//drawing :::::::::::::::
					{
					BSP_LCD_DrawPixel(40+i, 259, PBAR_COLOR_2);	
					BSP_LCD_DrawPixel(40+i, 262, PBAR_COLOR_2);	
					}
				else
					{
					BSP_LCD_DrawPixel(40+i, 259, PBAR_COLOR_3);	
					BSP_LCD_DrawPixel(40+i, 262, PBAR_COLOR_3);		
					}	
				}	
			else if(i>previous_position_bar+2)
				{
				if(i%2==0)							//drawing ||||||||||||||||||
					{
					ForceDrawVLine(i+40, 259, 4, LCD_COLOR_WHITE);		
					}
				else
					{
					ForceDrawVLine(i+40, 259, 4, PBAR_COLOR_1);		
					}
				}	
			}
		if(previous_position_bar>1)
			{
			ForceDrawVLine(40, 259, 4, PBAR_COLOR_2);						//gray scroll vertical line in start				
			}		
		DRAWN_IN_REMAIN = 1;	
		return;	
		}	
	else if(Tpos==DRAW_NEW_STATIC_WAVEFORM)						//Draw new static waveform		
		{
		previous_position_bar = 0;	
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(40, 230, 400, 40);				
		for(i=0;i<400;i++)						
			{			
			ForceDrawVLine(i+40, 256-(WFORMSTATIC[i]&0x1F), (WFORMSTATIC[i]&0x1F)+1, WS_COLOR_MAP[UTILITY_SETTINGS[7]][WFORMSTATIC[i]>>7][(WFORMSTATIC[i]>>2)&0x07]);		
			if(REMAIN_ENABLE)
				{
				if(i%2==0)
					{
					ForceDrawVLine(i+40, 259, 4, LCD_COLOR_WHITE);		
					}
				else
					{
					ForceDrawVLine(i+40, 259, 4, PBAR_COLOR_1);		
					}
				}
			else
				{
				if(i%2==0)
					{
					BSP_LCD_DrawPixel(40+i, 259, PBAR_COLOR_2);	
					BSP_LCD_DrawPixel(40+i, 262, PBAR_COLOR_2);	
					}
				else
					{
					BSP_LCD_DrawPixel(40+i, 259, PBAR_COLOR_3);	
					BSP_LCD_DrawPixel(40+i, 262, PBAR_COLOR_3);		
					}
				}
			}
		if(REMAIN_ENABLE==0)	
			{
			ForceDrawVLine(40, 259, 4, PBAR_COLOR_2);						//gray scroll vertical line in start	
			ForceDrawVLine(439, 259, 4, PBAR_COLOR_2);					//gray scroll vertical line in end	
			}
		}
	else if(Tpos==CLEAR_WAVEFORM_ARRAY)										//clear waveform array
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(40, 220, 400, 51);			
		}
	else if(Tpos==MS_NOT_LOADED)
		{
		BSP_LCD_SetFont(&Font15P);	
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf((char*)Buf, "%s", "NOT LOADED");	
		BSP_LCD_DisplayStringAt(193,239,Buf, LEFT_MODE);
		}
	else if(Tpos>MS_ERROR)													//Error message
		{
		BSP_LCD_SetFont(&Font15P);	
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf((char*)Buf, "%s", "ERROR");	
		BSP_LCD_DisplayStringAt(193,239,Buf, LEFT_MODE);
		sprintf((char*)Buf, "%2lu", (Tpos-MS_ERROR));			//ERROR NUMBER
		BSP_LCD_DisplayStringAt(252,239,Buf, LEFT_MODE);			
		}
	}


//function drawing range display
// 0 - 6%			0
// 1 - 10%		1
// 2 - 16%		2
// 3 - WIDE		3
void DrawTempoRange(uint8_t Range)
	{	
	BSP_LCD_SetFont(&FontBMP);
	if(Range==3)
		{
		BSP_LCD_SetTextColor(0xFFFF0000);
		}
	else
		{
		BSP_LCD_SetTextColor(0x30FF0000);	
		}
	sprintf((char*)Buf, "%s", "8");						//WIDE
	BSP_LCD_DisplayStringAt(445,227,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "9");						//WIDE
	BSP_LCD_DisplayStringAt(455,227,Buf, TRANSPARENT_MODE);	
		
	if(Range==2)
		{
		BSP_LCD_SetTextColor(0xFFFFFFFF);
		}
	else
		{
		BSP_LCD_SetTextColor(0x30FFFFFF);	
		}
	sprintf((char*)Buf, "%s", ")");						//+-16%
	BSP_LCD_DisplayStringAt(445,238,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "*");						//+-16%
	BSP_LCD_DisplayStringAt(455,238,Buf, TRANSPARENT_MODE);
		
	if(Range==1)
		{
		BSP_LCD_SetTextColor(0xFFFF0000);
		}
	else
		{
		BSP_LCD_SetTextColor(0x30FF0000);	
		}	
	sprintf((char*)Buf, "%s", "'");						//+-10%
	BSP_LCD_DisplayStringAt(445,249,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "(");						//+-10%
	BSP_LCD_DisplayStringAt(455,249,Buf, TRANSPARENT_MODE);	
	
	if(Range==0)
		{
		BSP_LCD_SetTextColor(0xFF00FF00);
		}
	else
		{
		BSP_LCD_SetTextColor(0x3000FF00);	
		}
	sprintf((char*)Buf, "%s", "+");						//+-6%
	BSP_LCD_DisplayStringAt(445,260,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", ",");						//+-6%	
	BSP_LCD_DisplayStringAt(455,260,Buf, TRANSPARENT_MODE);	
	}



//function drawing Memory and Cue Pyramids
// 0 - nothing
// 1 - only Cue Pyramids
// 2 - only Memory Pyramids
// 3 - Memory and Cue Pyramids	
void DrawMemoryCuePyramid(uint8_t draw)
	{
	BSP_LCD_SetFont(&FontBMP);	
	if(draw==0 || draw==2)
		{
		BSP_LCD_SetTextColor(0xFF000000);	
		}
	else
		{
		BSP_LCD_SetTextColor(0xFFFF0000);		
		}
	sprintf((char*)Buf, "%s", " ");		//pyramid CUE		
	BSP_LCD_DisplayStringAt(1,256,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "!");		//pyramid CUE		
	BSP_LCD_DisplayStringAt(13,256,Buf, TRANSPARENT_MODE);
	
	if(draw==0 || draw==2)
		{
		BSP_LCD_SetTextColor(0xFF000000);	
		}
	else
		{
		BSP_LCD_SetTextColor(0xFF8080FF);		
		}
	sprintf((char*)Buf, "%s", " ");		//pyramid CUE		
	BSP_LCD_DisplayStringAt(1,155,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "!");		//pyramid CUE		
	BSP_LCD_DisplayStringAt(13,155,Buf, TRANSPARENT_MODE);
		
	
	if(draw==0 || draw==1)
		{
		BSP_LCD_SetTextColor(0xFF000000);	
		}
	else
		{
		BSP_LCD_SetTextColor(0xFFFF0000);		
		}
	sprintf((char*)Buf, "%s", "#");						//pyramid memory
	BSP_LCD_DisplayStringAt(1,230,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "$");						//pyramid memory
	BSP_LCD_DisplayStringAt(13,230,Buf, TRANSPARENT_MODE);
				
	if(draw==0 || draw==1)
		{
		BSP_LCD_SetTextColor(0xFF000000);	
		}
	else
		{
		BSP_LCD_SetTextColor(0xFF8080FF);		
		}
	sprintf((char*)Buf, "%s", "#");						//pyramid memory	
	BSP_LCD_DisplayStringAt(1, 84,Buf, TRANSPARENT_MODE);
	sprintf((char*)Buf, "%s", "$");						//pyramid memory	
	BSP_LCD_DisplayStringAt(13,84,Buf, TRANSPARENT_MODE);	
	}
	
void DrawMinuteMarkers(void)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);	
	uint32_t a = 3600000/all_long;
	uint32_t npoint = all_long/9000;
	while(npoint>0)
		{
		if(npoint*a<400)
			{
			BSP_LCD_DrawLine(40+npoint*a, 269, 40+npoint*a, 266);
			}	
		npoint--;		
		}
	return;	
	}
	
	
	
//draw ZOOMGRID picture	on display
void DrawZOOMGRID(void)
	{
	uint8_t clr;
	uint32_t color;	
	uint16_t n = 0;
	while(n<5148)						//draw picture
		{
		color = 0x0000FF00;	
		clr = (LAYER[n*2+1]>>2)<<3;
		color += clr;	
		color = color<<8;
		clr = (LAYER[n*2+1]<<6)+((LAYER[n*2]>>5)<<3);
		color += clr;
		color = color<<8;		
		clr = LAYER[n*2]<<3;
		color += clr;
		BSP_LCD_DrawPixel(441+n%36, (174-n/36), color);
		n++;		
		}
	return;	
	}

//	draw logo Pioneer DJ on display
void DrawLOGO(void)
	{		
	uint8_t frame = 0;
	uint8_t VL = 0;	
	uint16_t n = 0;
	while(frame<26)							//change frame
		{	
		n = 0;	
		if(VL==0)
			{
			BSP_LCD_SelectLayer(1);	
			while(n<57600)						//draw picture
				{
				BSP_LCD_DrawPixel(n%480, (215-n/480), (0xFF000000+ANIMATION[frame][n][1]+256*ANIMATION[frame][n][2]+65536*ANIMATION[frame][n][0]));
				n++;		
				}	
			BSP_LCD_SetTransparency(1, 255);		//верхний слой виден
			BSP_LCD_SelectLayer(0);
			VL = 1;
			}
		else
			{
			BSP_LCD_SelectLayer(0);
			while(n<57600)						//draw picture
				{
				BSP_LCD_DrawPixel(n%480, (215-n/480), (0xFF000000+ANIMATION[frame][n][1]+256*ANIMATION[frame][n][2]+65536*ANIMATION[frame][n][0]));
				n++;		
				}		
			BSP_LCD_SetTransparency(1, 0);		//верхний не слой виден
			VL = 0;	
			}
		frame++;
		HAL_Delay(15);	
		}
	BSP_LCD_SetTransparency(1, 0);		//верхний не слой виден	
	BSP_LCD_SelectLayer(0);	
		
	HAL_Delay(330);	
	BSP_LCD_SetFont(&Font15P);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	sprintf((char*)Buf, "%s", "ver.");
	BSP_LCD_DisplayStringAt(384,257,Buf, LEFT_MODE);	
	BSP_LCD_DisplayStringAt(420,257,FIRMWARE_VERSION, LEFT_MODE);	
			
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);	
	BSP_LCD_DrawLine(101, 190, 379, 190);	
	BSP_LCD_DrawLine(101, 196, 379, 196);
	BSP_LCD_DrawLine(100, 191, 100, 195);
	BSP_LCD_DrawLine(380, 191, 380, 195);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	n = 0;	
	while(n<277)
		{
		BSP_LCD_DrawLine(102+n, 192, 102+n, 194);
		n++;	
		HAL_Delay(11);	
		}	
	HAL_Delay(600);		
	return;	
	}	

///////////////////////////////////	
//
//	Draw REKORDBOX logo on Display
//	
void DrawREKORDBOX(void)
	{		
	uint16_t n = 0;
	
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawLine(240,86,240,171);
	BSP_LCD_DrawLine(241,86,241,171);			
	while(n<11264)						//draw picture
		{
		BSP_LCD_DrawPixel((112+(n&0xFF)), (146-(n>>8)), (0xFF000000+REKBX[n][1]+256*REKBX[n][2]+65536*REKBX[n][0]));
		n++;		
		}
	BSP_LCD_SetTransparency(1, 255);		//верхний слой виден
	BSP_LCD_SelectLayer(0);		
	return;	
	}	
	
	
	
//Show NEEDLE on display	
void ShowNEEDLE(uint8_t needle)
	{
	if(needle)
		{	
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
		}
	BSP_LCD_SetFont(&FontBMP);
	sprintf((char*)Buf, "%s", "45");						//NEEDLE
	BSP_LCD_DisplayStringAt(142, 185, Buf, TRANSPARENT_MODE);	
	}

//Show tempo func
//
//
void ShowTempo(uint16_t tempo)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font23D);

	if(tempo==10000)	
		{
		sprintf((char*)Buf, "%s", " ");			//NONE	
		}	
	else if(tempo>10000)
		{
		sprintf((char*)Buf, "%s", "+");			//PLUS	
		}
	else
		{
		sprintf((char*)Buf, "%s", "-");			//MINUS
		}

	if(tempo>10000)	
		{
		tempo = tempo - 10000;	
		}
	else
		{
		tempo = 10000-tempo;	
		}
		
	BSP_LCD_DisplayStringAt(323,194,Buf, LEFT_MODE);
	sprintf((char*)Buf, "%s", ".");			//dot for pitch
	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(364, 199, 19, 18);	
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

	if(tempo_range==3)							//wide range
		{
		BSP_LCD_DisplayStringAt(379,194,Buf, TRANSPARENT_MODE);
		BSP_LCD_SetFont(&Font18D);
		sprintf((char*)Buf, "%3lu", tempo/100);			//PITCH
		BSP_LCD_DisplayStringAt(333, 199, Buf, LEFT_MODE);
		sprintf((char*)Buf, "%01lu", (tempo%100)/10);			//PITCH
		BSP_LCD_DisplayStringAt(384, 199, Buf, LEFT_MODE);	
		}
	else
		{
		BSP_LCD_DisplayStringAt(364,194,Buf, TRANSPARENT_MODE);
		BSP_LCD_SetFont(&Font18D);
		sprintf((char*)Buf, "%2lu", tempo/100);			//PITCH
		BSP_LCD_DisplayStringAt(333, 199, Buf, LEFT_MODE);	
		sprintf((char*)Buf, "%02lu", tempo%100);			//PITCH
		BSP_LCD_DisplayStringAt(369, 199, Buf, LEFT_MODE);		
		}
	}

//SHOW TRACK NUMBER
void ShowTrackNumber(uint8_t track)
	{
	track = track%100;	
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font26D);
	sprintf((char*)Buf, "%01lu", track/10);			//TRACK NUMBER x10
	BSP_LCD_DisplayStringAt(48,192,Buf, LEFT_MODE);
	sprintf((char*)Buf, "%01lu", track%10);			//TRACK NUMBER	x1
	BSP_LCD_DisplayStringAt(66,192,Buf, LEFT_MODE);	
	}

//Function redraw bar on static waveform and redraw dynamic waveform
//position = 1/150 sec
void RedrawWaveforms(uint32_t position)
	{
	if(position>all_long)
		{
		return;	
		}
	uint32_t clock_pos;	

	if(REMAIN_ENABLE)
		{
		clock_pos = all_long - position;	
		}	
	else
		{
		clock_pos	= position;
		}
		
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);	
	BSP_LCD_SetFont(&Font23D);
		

	if(forcibly_redraw==1)
		{
		Prev10m = 0xFF;
		Prev1m = 0xFF;
		Prev10s = 0xFF;
		Prev1s = 0xFF;
		Prev10f = 0xFF;
		Prev1f = 0xFF;
		PrevHf = 0xFF;				
		}

	if(Prev10m != (clock_pos/90000)%10)			
		{
		Prev10m = (clock_pos/90000)%10;	
		sprintf((char *)Buf , "%0lu", Prev10m);				//10 Min	
		BSP_LCD_DisplayStringAt(139, 194, Buf, LEFT_MODE);	
		}
	if(Prev1m != (clock_pos/9000)%10)			
		{
		Prev1m = (clock_pos/9000)%10;	
		sprintf((char *)Buf , "%0lu", Prev1m);				//1 Min	
		BSP_LCD_DisplayStringAt(156, 194, Buf, LEFT_MODE);				
		}
	if(Prev10s != (clock_pos/1500)%6)
		{
		Prev10s = (clock_pos/1500)%6;
		sprintf((char *)Buf , "%0lu", Prev10s);				//10 Sec	
		BSP_LCD_DisplayStringAt(198, 194, Buf, LEFT_MODE);		
		}
	if(Prev1s != (clock_pos/150)%10)
		{
		Prev1s = (clock_pos/150)%10;
		sprintf((char *)Buf , "%0lu", Prev1s);				//1 Sec	
		BSP_LCD_DisplayStringAt(215, 194, Buf, LEFT_MODE);
		}		
	if(Prev10f != ((clock_pos/2)%75)/10)
		{
		Prev10f = ((clock_pos/2)%75)/10;
		sprintf((char *)Buf , "%0lu", Prev10f);				//10 F	
		BSP_LCD_DisplayStringAt(247, 194, Buf, LEFT_MODE);
		}		
	if(Prev1f != ((clock_pos/2)%75)%10)
		{
		Prev1f = ((clock_pos/2)%75)%10;
		sprintf((char *)Buf , "%0lu", Prev1f);				//1 F	
		BSP_LCD_DisplayStringAt(264, 194, Buf, LEFT_MODE);	
		}
	if(PrevHf != clock_pos%2)
		{
		PrevHf = clock_pos%2;
		BSP_LCD_SetFont(&Font18D);		
		if(PrevHf%2==1)
			{
			sprintf((char*)Buf, "%s", "5");	
			}
		else
			{
			sprintf((char*)Buf, "%s", "0");	
			}
		BSP_LCD_DisplayStringAt(291, 199, Buf, LEFT_MODE);					
		}		
		
	if(needle_enable || (Tbuffer[23]&0x20))						//detecting touch on sensor or touch on jog 
		{
		if(RED_VERTICAL_LINE==0)
			{
			RED_VERTICAL_LINE = 1;	
			forcibly_redraw = 1;	
			}
		}
	else
		{
		if(RED_VERTICAL_LINE)
			{
			RED_VERTICAL_LINE = 0;	
			forcibly_redraw = 1;	
			}
		}

	DrawStaticWFM(position*399/all_long);	
		
	if(dSHOW==WAVEFORM)
		{
		position = position/DynamicWaveformZOOM;					//zoom correction	
		if(position != PreviousPositionDW || forcibly_redraw==1)
			{
			PreviousPositionDW = position;
			if(VisibleLayer==0)
				{
				BSP_LCD_SelectLayer(1);	
				intDRAW_WAVEFORM_FRAME(position);	
				BSP_LCD_SetTransparency(1, 255);		//верхний слой виден
				BSP_LCD_SelectLayer(0);
				VisibleLayer = 1;
				}
			else
				{
				BSP_LCD_SelectLayer(0);
				intDRAW_WAVEFORM_FRAME(position);	
				BSP_LCD_SetTransparency(1, 0);		//верхний не слой виден
				VisibleLayer = 0;	
				}
			ShowPhaseMeter(bars);
			if(originalBPM != BPMGRID[bars])										//вынести проверку за пределы (dSHOW==WAVEFORM)!!!
				{
				originalBPM = BPMGRID[bars];
				tempo_need_update = 2;		
				}	
			}
		}	
	forcibly_redraw = 0;
	return;	
	}
//////////////////////////////////////////////	
//	
//	internal function for redraw waveform	
//	
void intDRAW_WAVEFORM_FRAME(uint32_t position)
	{
	uint32_t i, adr, BG_COLOR;
	uint16_t u, x; 	
	uint8_t	j;
	x = 0;
	u = 0;	
				
	if(position>=200)
		{
		while((BEATGRID[u]-(BEATGRID[u]%DynamicWaveformZOOM))<(DynamicWaveformZOOM*(position-200)))
			{
			u++;	
			}	
		}	
	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
	BSP_LCD_FillRect(36, 79, 408, 5);
	BSP_LCD_FillRect(36, 166, 408, 5);				
	BSP_LCD_FillRect(40, 84, 400, 5);
	BSP_LCD_FillRect(40, 161, 400, 5);
			
	for(i=0;i<400;i++)
		{	
		adr = DynamicWaveformZOOM*(i+position-200);	

		if(CUE_ADR<LOOP_OUT)	
			{
			if(adr>=CUE_ADR && adr<LOOP_OUT)	
				{
				if(loop_active)
					{
					BG_COLOR = LOOP_ACTIVE_COLOR;	
					}
				else
					{
					BG_COLOR = LOOP_INACTIVE_COLOR;		
					}		
				}
			else
				{
				BG_COLOR = LCD_COLOR_BLACK;		
				}			
			}
		else
			{
			BG_COLOR = LCD_COLOR_BLACK;	
			}
	
		if(adr<=all_long)
			{
			if(number_of_memory_cue_points>0)					//Draw MEMORY CUE triangle
				{
				for(j=0;j<number_of_memory_cue_points;j++)
					{
					if((MEMORY_adr[0][j]-(MEMORY_adr[0][j]%DynamicWaveformZOOM))==adr)
						{
						BSP_LCD_SetTextColor(LCD_COLOR_RED);
						FillTriangle(i+36, i+44, i+40, 79, 79, 83);	
						}	
					}
				}	
			if(number_of_hot_cue_points>0)				//Draw HCUE triangle
				{
				BSP_LCD_SetFont(&Font8);	
				Buf[1] = 0;		
				for(j=0;j<3;j++)
					{
					if(HCUE_adr[0][j]!=0xFFFF)							
						{
						if((HCUE_adr[0][j]-(HCUE_adr[0][j]%DynamicWaveformZOOM))==adr)
							{
							if(HCUE_type[j]&0x1)
								{
								BSP_LCD_SetTextColor(CUE_COLOR);	
								}
							else
								{
								BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
								}
//							if(i<397)
//								{
//								Buf[0] = 35+j;
//								BSP_LCD_DisplayStringAt(i+45, 76, Buf, TRANSPARENT_MODE);	
//								BSP_LCD_DisplayStringAt(i+45, 167, Buf, TRANSPARENT_MODE);	
//								}
							FillTriangle(i+36, i+44, i+40, 79, 79, 83);
							FillTriangle(i+36, i+44, i+40, 170, 170, 166);
							}
						}
					}
				}
			if((CUE_ADR-(CUE_ADR%DynamicWaveformZOOM))==adr) //Draw CUE triangle
				{
				BSP_LCD_SetTextColor(CUE_COLOR);
				FillTriangle(i+36, i+44, i+40, 170, 170, 166);
				}
				
			if((BEATGRID[u+x]-(BEATGRID[u+x]%DynamicWaveformZOOM))==adr)	
				{
				if(((u+x)%4)==((1-GRID_OFFSET)&0x03))					//red grid
					{
					ForceDrawVLine(i+40, 84, 5, LCD_COLOR_RED);
					ForceDrawVLine(i+40, 161, 5, LCD_COLOR_RED);
					}
				else if(DynamicWaveformZOOM<8)			//white grid
					{	
					ForceDrawVLine(i+40, 84, 5, LCD_COLOR_WHITE);
					ForceDrawVLine(i+40, 161, 5, LCD_COLOR_WHITE);		
					}
				x++;	
				}	
			}

		if(i==200)																		///you can optimize 1 raz draw red line
			{
			bars = u+x;		
			}
		else if(i==201)
			{
			}
		else
			{
			if(adr<=all_long)
				{
				if(DynamicWaveformZOOM==1)
					{	
					ForceDrawVLine(i+40, 124-(WFORMDYNAMIC[adr]&0x1F), 2+2*(WFORMDYNAMIC[adr]&0x1F), COLOR_MAP[UTILITY_SETTINGS[7]][WFORMDYNAMIC[adr]>>5]);		//124-125px center	
					ForceDrawVLine(i+40, 92, 32-(WFORMDYNAMIC[adr]&0x1F), BG_COLOR);								
					ForceDrawVLine(i+40, 126+(WFORMDYNAMIC[adr]&0x1F), 32-(WFORMDYNAMIC[adr]&0x1F), BG_COLOR);			
					}
				else 	
					{
					uint8_t amplitude = (WFORMDYNAMIC[adr]&0x1F);
					uint8_t color = (WFORMDYNAMIC[adr]>>5);
					for(j=0;j<(DynamicWaveformZOOM-1);j++)
						{
						if((WFORMDYNAMIC[adr+j+1]&0x1F)>amplitude)
							{
							amplitude	= (WFORMDYNAMIC[adr+j+1]&0x1F);
							if(amplitude>17)
								{
								color = (WFORMDYNAMIC[adr+j+1]>>5);
								}
							}
						}		
					ForceDrawVLine(i+40, 124-amplitude, 2+2*amplitude, COLOR_MAP[UTILITY_SETTINGS[7]][color]);		//124-125px center
					ForceDrawVLine(i+40, 92, 32-amplitude, BG_COLOR);	
					ForceDrawVLine(i+40, 126+amplitude, 32-amplitude, BG_COLOR);		
					}
				}
			else
				{
				ForceDrawVLine(i+40, 92, 66, LCD_COLOR_BLACK);		
				}	
			}			
		}
		
		
	if(RED_VERTICAL_LINE)							//detecting touch on sensor or touch on jog 
		{
		ForceDrawVLine(240, 76, 98, LCD_COLOR_RED);			//red scroll
		ForceDrawVLine(241, 76, 98, LCD_COLOR_RED);		
		}
	else
		{
		ForceDrawVLine(240, 76, 98, LCD_COLOR_WHITE);	
		ForceDrawVLine(241, 76, 98, LCD_COLOR_WHITE);		
		}	
	return;	
	}
	

//Draw CUE orange triangle on time bar	
// p=0 - disable triangle	
void DrawCueMarker(uint16_t p)
	{
	if(p<401)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(37, 266, 406, 4);
		DrawMinuteMarkers();
		if(p!=0)
			{
			BSP_LCD_SetTextColor(CUE_COLOR);
			FillTriangle(p+36, p+42, p+39, 269, 269, 266);
			}	
		}	
	}

////////////////////////////////////////////////	
//Draw MEMORY red triangle on time bar	
// p=0 - disable all triangle	
//#define NONE_MARK				0
//#define MEMORY_MARK			1
//#define HOT_CUE_A_MARK	2
//#define HOT_CUE_B_MARK	3
//#define HOT_CUE_C_MARK	4
void DrawMemoryMarker(uint16_t p, uint8_t type, uint32_t color)
	{
	if(type==NONE_MARK)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(37, 223, 406, 7);
		return;
		}
	else if(p<401)
		{
		if(p!=0)
			{
			BSP_LCD_SetTextColor(color);
			if(type>1)
				{
				BSP_LCD_SetFont(&Font8);	
				Buf[1] = 0;	
				Buf[0] = 33+type;
				BSP_LCD_DisplayStringAt(p+44, 223, Buf, TRANSPARENT_MODE);		
				FillTriangle(p+36, p+42, p+39, 226, 226, 229);
				//FillTriangle(p+36, p+42, p+39, 269, 269, 266);				//перенести выполнение этой функции за пределы функции LOAD TRACK. Выполнять ее после DrawMinuteMarker!
				}
			else
				{	
				FillTriangle(p+36, p+42, p+39, 226, 226, 229);
				}	
			}	
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_FillRect(37, 226, 406, 4);
			}
		return;
		}	
	}	
	
//init DAC on board discovery 
//and start SAI transaction
void DAC_Init_and_Start(uint8_t volume)
	{
	uint8_t _tmp[2];
	uint32_t deviceid = 0x00;
	AUDIO_DrvTypeDef          *audio_drv;
	HAL_SAI_Transmit(&hsai_BlockA2, _tmp, 2, 200);
  deviceid = wm8994_drv.ReadID(AUDIO_I2C_ADDRESS);
  if((deviceid) == WM8994_ID)
		{  
		wm8994_Reset(AUDIO_I2C_ADDRESS);
    audio_drv = &wm8994_drv; 
    audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_BOTH, volume, AUDIO_FREQUENCY_44K);
		wm8994_SetMute(AUDIO_I2C_ADDRESS, AUDIO_MUTE_OFF);	
		}
	HAL_SAI_Transmit_IT(&hsai_BlockA2, _tmp, 2);
	return;	
	}
	
	
//Rekordbox database parser
//Functions:
//Open and read file export.pdb
//find all tracks
//write trackname in massive
//write adress position datatrack in file export.pdb
//return number of tracks or 0 if error enable 	
uint16_t DATABASE_PARSER(void)
	{	
	res = f_open(&file, path_export, FA_READ);
	if (res != FR_OK)
		{
		return 0;	
		}		
	uint32_t FILSIZE = f_size(&file);
	char str[4] = ".DAT";
	uint32_t cursor = 0;
	uint32_t BPM_CNT = 0;
	uint8_t	cycle_en;
	uint16_t all_trks = 0;			//512 max
	uint16_t T_ID = 0;
	uint16_t i  = 0;	
		
	uint32_t NEXT_PAGE_0, LAST_PAGE_0, P_PAGE_0;	
	uint32_t NEXT_PAGE_5, LAST_PAGE_5; //page for KEY
	uint32_t NEXT_PAGE_7, LAST_PAGE_7; //page for PLAYLIST tree
	uint32_t NEXT_PAGE_8, LAST_PAGE_8; //page for PLAYLIST entries		
	uint32_t NEXT_PAGE_19, LAST_PAGE_19; //page for history, flash name, date
		
	for(T_ID=0;T_ID<1024;T_ID++)
		{
		WFORMDYNAMIC[4096+T_ID] = 0xFF;				//give out buffer for track ID in WFORMDYNAMIC	offset 8192
		}
	T_ID = 0;	
		
	res = f_read(&file, WFORMDYNAMIC, 4096, &nbytes);
	if (res != FR_OK)
		{
		return 0;	
		}	
		
	cursor = 28;		
	while(cursor<(28+16*(WFORMDYNAMIC[8] + 256*WFORMDYNAMIC[9] + 65536*WFORMDYNAMIC[10] + 16777216*WFORMDYNAMIC[11])))	//scan first page and search link for all type pages with tables 
		{
		if(WFORMDYNAMIC[cursor]==0 && 
			 WFORMDYNAMIC[cursor+1]==0 && 
			 WFORMDYNAMIC[cursor+2]==0 && 
			 WFORMDYNAMIC[cursor+3]==0)	//Track metadata: title, artist, genre, artwork ID, playing time, etc
			{
			NEXT_PAGE_0 = WFORMDYNAMIC[cursor+8] + 256*WFORMDYNAMIC[cursor+9] + 65536*WFORMDYNAMIC[cursor+10] + 16777216*WFORMDYNAMIC[cursor+11];
			LAST_PAGE_0 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];
			}
		else if(WFORMDYNAMIC[cursor]==5 && 
						WFORMDYNAMIC[cursor+1]==0 && 
						WFORMDYNAMIC[cursor+2]==0 && 
						WFORMDYNAMIC[cursor+3]==0)	//Musical keys, for reference by tracks, searching, and key matching
			{
			NEXT_PAGE_5 = WFORMDYNAMIC[cursor+8] + 256*WFORMDYNAMIC[cursor+9] + 65536*WFORMDYNAMIC[cursor+10] + 16777216*WFORMDYNAMIC[cursor+11];	
			LAST_PAGE_5 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];	
			}	
		else if(WFORMDYNAMIC[cursor]==7 && 
						WFORMDYNAMIC[cursor+1]==0 && 
						WFORMDYNAMIC[cursor+2]==0 && 
						WFORMDYNAMIC[cursor+3]==0)	//Describes the hierarchical tree structure of available playlists and folders grouping them
			{
			NEXT_PAGE_7 = WFORMDYNAMIC[cursor+8] + 256*WFORMDYNAMIC[cursor+9] + 65536*WFORMDYNAMIC[cursor+10] + 16777216*WFORMDYNAMIC[cursor+11];	
			LAST_PAGE_7 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];		
			}		
		else if(WFORMDYNAMIC[cursor]==8 && 
						WFORMDYNAMIC[cursor+1]==0 && 
						WFORMDYNAMIC[cursor+2]==0 && 
						WFORMDYNAMIC[cursor+3]==0)	//Links tracks to playlists, in the right order
			{
			NEXT_PAGE_8 = WFORMDYNAMIC[cursor+8] + 256*WFORMDYNAMIC[cursor+9] + 65536*WFORMDYNAMIC[cursor+10] + 16777216*WFORMDYNAMIC[cursor+11];	
			LAST_PAGE_8 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];		
			}	
		else if(WFORMDYNAMIC[cursor]==19 && 
						WFORMDYNAMIC[cursor+1]==0 && 
						WFORMDYNAMIC[cursor+2]==0 && 
						WFORMDYNAMIC[cursor+3]==0)	//Links tracks to playlists, in the right order
			{
			NEXT_PAGE_19 = WFORMDYNAMIC[cursor+8] + 256*WFORMDYNAMIC[cursor+9] + 65536*WFORMDYNAMIC[cursor+10] + 16777216*WFORMDYNAMIC[cursor+11];	
			LAST_PAGE_19 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];		
			}		
		cursor+=16;		
		}
	cursor = 0;
		
	f_lseek(&file, 4096*NEXT_PAGE_0); 				//first page with track metadata: title, artist, genre, artwork ID, playing time, etc
	
	while(NEXT_PAGE_0<(FILSIZE>>12) && LAST_PAGE_0>=NEXT_PAGE_0)	
		{
		res = f_read(&file, WFORMDYNAMIC, 4096, &nbytes);
		if (res != FR_OK)
			{
			return 0;	
			}
		cursor = 0;
		P_PAGE_0 = NEXT_PAGE_0;
		NEXT_PAGE_0 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];		
			
		if(WFORMDYNAMIC[8]==0 && WFORMDYNAMIC[9]==0 && WFORMDYNAMIC[10]==0 && WFORMDYNAMIC[11]==0 && (WFORMDYNAMIC[27]&0x40)==0 && WFORMDYNAMIC[36]==0 && WFORMDYNAMIC[37]==0)		//check page type
			{			
			while(cursor<4095)
				{
				if(WFORMDYNAMIC[cursor]==0x24 && WFORMDYNAMIC[cursor+1]==0)
					{	
					T_ID = WFORMDYNAMIC[cursor+72] + 256*WFORMDYNAMIC[cursor+73];		
						
					if(T_ID>512)
						{
						return 0xFFFF;	
						}
						
					if(WFORMDYNAMIC[4096+2*(T_ID-1)]==0xFF && WFORMDYNAMIC[4097+2*(T_ID-1)]==0xFF)		//check track new or 
						{
						all_trks++;	
						}	
					WFORMDYNAMIC[4096+2*(T_ID-1)] = WFORMDYNAMIC[cursor+72];	
					WFORMDYNAMIC[4097+2*(T_ID-1)] = WFORMDYNAMIC[cursor+73];		
					cursor+= 32;	
					key_id[T_ID-1] = WFORMDYNAMIC[cursor];
					cursor+= 24;					
					BPM_CNT = WFORMDYNAMIC[cursor] + 256*WFORMDYNAMIC[cursor+1] + 65535*WFORMDYNAMIC[cursor+2];	
					BPM_CNT = BPM_CNT/10;	
					original_tempo[T_ID-1] = BPM_CNT;
					cursor+= 28;	
					duration[T_ID-1] = WFORMDYNAMIC[cursor] + 256*WFORMDYNAMIC[cursor+1];	
					cursor+= 4;
					rating[T_ID-1] = WFORMDYNAMIC[cursor] + 256*WFORMDYNAMIC[cursor+1];
					cursor+= 4;						
					cycle_en = 1;	
					while(cursor<4095 && cycle_en)
						{	
						if(WFORMDYNAMIC[cursor]==str[1])
							{
							if(WFORMDYNAMIC[cursor-1]==str[0] & 
								 WFORMDYNAMIC[cursor+1]==str[2] & 
								 WFORMDYNAMIC[cursor+2]==str[3])				//".DAT" Finded!
								{
								parcser_adress[T_ID-1] = (4096*P_PAGE_0)+cursor;										//save position ".[D]AT"	
								cursor = cursor+16;									
								for(i=0;((i<54) && (WFORMDYNAMIC[cursor+i] != 3));i++)						//copy track name	
									{		
									playlist[T_ID-1][i] = WFORMDYNAMIC[cursor+i];
									}

								while(i<54)
									{
									playlist[T_ID-1][i] = 32;
									i++;		
									}									
								playlist[T_ID-1][53] = 0;
								playlist[T_ID-1][54] = 0;	
									
								i = 0;	
								cursor+= 10;	
								cycle_en = 0;	
								}
							}
						cursor++;	
						}	
					}
				cursor++;	
				}				
			}
		f_lseek(&file, 4096*NEXT_PAGE_0);
		}	
	
	f_lseek(&file, 4096*NEXT_PAGE_5); 				//first page with musical keys, for reference by tracks, searching, and key matching
		
	cycle_en = 1;		
	while(NEXT_PAGE_5<(FILSIZE>>12) && cycle_en)	
		{
		if(LAST_PAGE_5==NEXT_PAGE_5)			//scan last page
			{
			cycle_en = 0;	
			}
		res = f_read(&file, WFORMDYNAMIC, 4096, &nbytes);
		if (res != FR_OK)
			{
			return 0;	
			}
		cursor = 0;
			
		NEXT_PAGE_5 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];						
		if(WFORMDYNAMIC[8]==5 && WFORMDYNAMIC[9]==0 && WFORMDYNAMIC[10]==0 && WFORMDYNAMIC[11]==0 && (WFORMDYNAMIC[27]&0x40)==0 && WFORMDYNAMIC[36]==0 && WFORMDYNAMIC[37]==0 && WFORMDYNAMIC[24]!=0)		//check page type
			{	
			P_PAGE_0 = WFORMDYNAMIC[24];	
			cursor = 40;	
			while(P_PAGE_0>0)	
				{					
				KEYS[WFORMDYNAMIC[cursor]-1][0] = WFORMDYNAMIC[cursor+9];
				KEYS[WFORMDYNAMIC[cursor]-1][1] = WFORMDYNAMIC[cursor+10];	
				KEYS[WFORMDYNAMIC[cursor]-1][2] = WFORMDYNAMIC[cursor+11];
				KEYS[WFORMDYNAMIC[cursor]-1][3] = WFORMDYNAMIC[cursor+12];
				if(KEYS[WFORMDYNAMIC[cursor]-1][3]<33 || KEYS[WFORMDYNAMIC[cursor]-1][3]>125)
					{
					KEYS[WFORMDYNAMIC[cursor]-1][3] = 0;	
					}
						
				if(WFORMDYNAMIC[cursor+8]==5)
					{
					KEYS[WFORMDYNAMIC[cursor]-1][1] = 0;
					cursor+=12;	
					}
				else if(WFORMDYNAMIC[cursor+8]==7)
					{
					KEYS[WFORMDYNAMIC[cursor]-1][2] = 0;
					cursor+=12;	
					}	
				else if(WFORMDYNAMIC[cursor+8]==9)
					{
					KEYS[WFORMDYNAMIC[cursor]-1][3] = 0;
					cursor+=12;	
					}
				else
					{	
					cursor = cursor + 12 + 4*((((WFORMDYNAMIC[cursor+8]-1)/2)-1)/4);		
					}
				P_PAGE_0--;	
				}
			}
			
		f_lseek(&file, 4096*NEXT_PAGE_5);	
		}
		
	f_lseek(&file, 4096*NEXT_PAGE_7); 				//first page for PLAYLIST tree
	
	cycle_en = 1;		
	for(i=0;i<4200;i++)					//prepare 0 buffer
		{
		WFORMDYNAMIC[8192+i] = 0;	
		}		
	while(LAST_PAGE_7<(FILSIZE>>12) && cycle_en)	
		{
		if(LAST_PAGE_7==NEXT_PAGE_7)			//scan last page
			{
			cycle_en = 0;	
			}
		res = f_read(&file, WFORMDYNAMIC, 4096, &nbytes);
		if (res != FR_OK)
			{
			return 0;	
			}
			
		cursor = 0;
	
		NEXT_PAGE_7 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];			
		if(WFORMDYNAMIC[8]==7 && WFORMDYNAMIC[9]==0 && WFORMDYNAMIC[10]==0 && WFORMDYNAMIC[11]==0 && (WFORMDYNAMIC[27]&0x40)==0 && WFORMDYNAMIC[36]==0 && WFORMDYNAMIC[37]==0 && WFORMDYNAMIC[24]!=0)		//check page type
			{	
			P_PAGE_0 = WFORMDYNAMIC[24];
			cursor = 40;	
			P_PAGE_0 = P_PAGE_0 + 1;		
			while(P_PAGE_0>0)	
				{		
				BPM_CNT = (((WFORMDYNAMIC[cursor+20]-1)/2)-1);		//playlist lenght name	
				if(WFORMDYNAMIC[cursor+16]==0 && WFORMDYNAMIC[cursor+17]==0 && WFORMDYNAMIC[cursor+18]==0 && WFORMDYNAMIC[cursor+19]==0)			//row is playlist
					{
					//WFORMDYNAMIC[cursor+12]			//playlist ID		
					for(i=0;(i<BPM_CNT && i<20); i++)
						{
						WFORMDYNAMIC[8192+21*(WFORMDYNAMIC[cursor+12]-1)+i] = WFORMDYNAMIC[cursor+21+i];	
						}
					WFORMDYNAMIC[8192+21*(WFORMDYNAMIC[cursor+12]-1)+20] = WFORMDYNAMIC[cursor+12];			//playlist ID	 	
					}					
				cursor = cursor + 24 + 4*(BPM_CNT/4);
				P_PAGE_0--;	
				}
			}			
		f_lseek(&file, 4096*NEXT_PAGE_7);	
		}

	TOTAL_TRACKLISTS = 0;
	uint8_t inp = 0;
	uint8_t cnt;	
	for(BPM_CNT=1;BPM_CNT<255;BPM_CNT++)	
		{
		for(i=0; i<200;i++)
			{
			if(WFORMDYNAMIC[8192+20+21*i]==BPM_CNT)				//have a playlist
				{
				for(cnt=0;cnt<21;cnt++)
					{
					TRACKLIST_NAME[TOTAL_TRACKLISTS][cnt] = WFORMDYNAMIC[8192+cnt+21*i];
					}
				inp = 1;	
				}
			}
		if(inp)	
			{
			if(TOTAL_TRACKLISTS<20)
				{
				TOTAL_TRACKLISTS++;
				}
			inp = 0;	
			}	
		}	
			
	f_lseek(&file, 4096*NEXT_PAGE_8); 				//first page for PLAYLIST tree
	
	cycle_en = 1;	
	for(i=0;i<20480;i++)					//prepare 0 buffer
		{
		WFORMDYNAMIC[8192+i] = 0xFF;	
		}		
		
	while(LAST_PAGE_8<(FILSIZE>>12) && cycle_en)	
		{
		if(LAST_PAGE_8==NEXT_PAGE_8)			//scan last page
			{
			cycle_en = 0;	
			}
		res = f_read(&file, WFORMDYNAMIC, 4096, &nbytes);
		if (res != FR_OK)
			{
			return 0;	
			}
		cursor = 0;
			
		NEXT_PAGE_8 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];			
		if(WFORMDYNAMIC[8]==8 && WFORMDYNAMIC[9]==0 && WFORMDYNAMIC[10]==0 && WFORMDYNAMIC[11]==0 && (WFORMDYNAMIC[27]&0x40)==0 && WFORMDYNAMIC[36]==0 && WFORMDYNAMIC[37]==0 && (WFORMDYNAMIC[24]!=0 || WFORMDYNAMIC[25]!=0))		//check page type
			{	
			P_PAGE_0 = WFORMDYNAMIC[34] + 256*WFORMDYNAMIC[35];
			cursor = 40;	
			P_PAGE_0 = P_PAGE_0 + 1;	
			while(P_PAGE_0>0)	
				{
				i = PlaylistID_to_Pos(WFORMDYNAMIC[cursor+8]);
				if(i==20)
					{
					return 0xFFFF;				//send to error
					}
				T_ID = i*1024+(2*(WFORMDYNAMIC[cursor]+256*WFORMDYNAMIC[cursor+1]-1));
				WFORMDYNAMIC[8192+T_ID] = WFORMDYNAMIC[cursor+4];			
				WFORMDYNAMIC[8193+T_ID] = WFORMDYNAMIC[cursor+5];	
				cursor+=12;
				P_PAGE_0--;	
				}
			}			
		f_lseek(&file, 4096*NEXT_PAGE_8);	
		}
				
	TRACKLIST_OFFSET[0] = 0;
	cursor = 0;	

			
	for(i=0;i<TOTAL_TRACKLISTS;i++)
		{
		TRACKLIST_NAME[i][20] = 0;
		cycle_en = 1;		
		for(BPM_CNT=0; (BPM_CNT<512); BPM_CNT++)
			{
			T_ID = (i*1024)+(2*BPM_CNT);
			if((WFORMDYNAMIC[8192+T_ID] + 256*WFORMDYNAMIC[8193+T_ID])==0xFFFF)
				{
				cycle_en = 0;
				}
			else
				{
				TRACKS_DATABASE[TRACKLIST_OFFSET[i]+BPM_CNT] = WFORMDYNAMIC[8192+T_ID] + 256*WFORMDYNAMIC[8193+T_ID]; 	
				cursor++;	
				}
			}
		TRACKLIST_OFFSET[i+1] = cursor;				
		}
		
	f_lseek(&file, 4096*NEXT_PAGE_19); 				//first page with history, flash name, date
		
	cycle_en = 1;		
	
	
	while(NEXT_PAGE_19<(FILSIZE>>12) && cycle_en)	
		{
		if(LAST_PAGE_19==NEXT_PAGE_19)			//scan last page
			{
			cycle_en = 0;	
			}
			
		res = f_read(&file, WFORMDYNAMIC, 4096, &nbytes);
		if (res != FR_OK)
			{
			return 0;	
			}
		cursor = 0;
			
		NEXT_PAGE_19 = WFORMDYNAMIC[cursor+12] + 256*WFORMDYNAMIC[cursor+13] + 65536*WFORMDYNAMIC[cursor+14] + 16777216*WFORMDYNAMIC[cursor+15];						
		if(WFORMDYNAMIC[8]==19 && WFORMDYNAMIC[9]==0 && WFORMDYNAMIC[10]==0 && WFORMDYNAMIC[11]==0 && (WFORMDYNAMIC[27]&0x40)==0 && WFORMDYNAMIC[36]==0 && WFORMDYNAMIC[37]==0 && WFORMDYNAMIC[24]!=0)		//check page type
			{	
			P_PAGE_0 = WFORMDYNAMIC[24];	
			cursor = 40;
			while(P_PAGE_0>0)	
				{
				if(WFORMDYNAMIC[cursor]==0x80 && WFORMDYNAMIC[cursor+1]==0x02)
					{
					BPM_CNT = cursor;	
					cursor+=12;
						
					T_ID = WFORMDYNAMIC[cursor]-1;			//date lenght sting
					T_ID = T_ID/2;
					T_ID-=1;						
					
					for(i=0;(i<T_ID && i<13);i++)
						{
						SD_DATE[i] = WFORMDYNAMIC[cursor+i+1];	
						}
					SD_DATE[i] = 0;		//end string	
					cursor = cursor + T_ID + 3;	
						
					T_ID = WFORMDYNAMIC[cursor]-1; 			//date lenght sting	
					T_ID = T_ID/2;
					T_ID-=1;		
					cursor = cursor + T_ID + 1;
					T_ID = WFORMDYNAMIC[cursor]-1; 			//date lenght sting	
					T_ID = T_ID/2;
					T_ID-=1;
							
					for(i=0;(i<T_ID && i<18);i++)	
						{
						SDCARD_NAME[i] = WFORMDYNAMIC[cursor+i+1];	
						}	
					SDCARD_NAME[i] = 0;		//end string		
					cursor = cursor+T_ID+6;	
					BPM_CNT = 4-((cursor - BPM_CNT)%4);
					cursor = cursor +	BPM_CNT;
					P_PAGE_0--;			
					}
				else
					{
					P_PAGE_0 = 0;	
					}
				}
			}
		f_lseek(&file, 4096*NEXT_PAGE_19);	
		}
		
	f_close(&file);					//Close file Export.pdb
	return all_trks;
	}

	
	

////////////////////////////////////////////////////////////////////////
//convert playlist ID to position in Tracklist name
//
//
///////////////////////////////////////////////////////////////////////	
uint8_t PlaylistID_to_Pos(uint8_t ID)
	{
	uint8_t K;
	for(K=0;K<20;K++)
		{
		if(ID==TRACKLIST_NAME[K][20])
			{
			return K;	
			}
		}
	return 20;
	}
	
	
////////////////////////////////////////////////////////////////////////
//	open export.pdb file, extract path for ANLZXXXX.DAT file
//	open ANLZXXXX.DAT file, extract path for audio file
//	extract static waveform data, bpm	
//	open ANLZXXXX.EXT file, extract dynamic waveform data, all_long data
//	open audio file
//
//	output: error code
////////////////////////////////////////////////////////////////////////	
uint16_t LOAD_TRACK(uint16_t TRACK_NUMBER)
		{
		if(TRACK_NUMBER==0)
			{
			return 1;				//invalid track number
			}
		uint16_t ERROR = 0;
		res = f_open(&file, path_export, FA_READ);
		if (res != FR_OK)
			{
			return 2;	//cannot open database
			}
		f_lseek(&file, (parcser_adress[TRACK_NUMBER-1]-42));			//find 0.DAT in file
		char path_ANLZ[45];
		res = f_read(&file, path_ANLZ, sizeof(path_ANLZ), &nbytes);
		if (res != FR_OK)
			{
			return 3;		//data not read
			}					
		f_close(&file);					//Close file Export.pdb
		path_ANLZ[0] = 48;
		path_ANLZ[1] = 58;	
		res = f_open(&file, path_ANLZ, FA_READ);
		if (res != FR_OK)
			{
			return 4;	//cannot open ANLZXXXX.DAT file
			}
		uint32_t FILSIZE = f_size(&file);
		if(FILSIZE>135000)
			{
			return 4;	
			}
		res = f_read(&file, WFORMDYNAMIC, FILSIZE, &nbytes);
		if (res != FR_OK)
			{
			return 5;	//cannot read ANLZXXXX.DAT file		
			}
		f_close(&file);					//Close file ANLZXXXX.DAT		
		uint32_t fsz;	
		fsz = WFORMDYNAMIC[8];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[9];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[10];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[11];
		if(fsz != FILSIZE)
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}
		fsz = WFORMDYNAMIC[4];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[5];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[6];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[7];
		uint32_t StPosHead = fsz;		
		if(WFORMDYNAMIC[StPosHead] != 80 |
			 WFORMDYNAMIC[StPosHead+1] != 80 |
			 WFORMDYNAMIC[StPosHead+2] != 84 | 
			 WFORMDYNAMIC[StPosHead+3] != 72)		//Check PPHT position in file
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}	
		fsz = WFORMDYNAMIC[StPosHead+4];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+5];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+6];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+7];								//head size
		uint32_t SPP = fsz+StPosHead+1;	
		fsz = WFORMDYNAMIC[StPosHead+12];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+13];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+14];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+15];							//Path Size
		char path_AUDIOTRACK[(fsz/2)+2];			//Create a Path for audiotrack
		uint16_t E=0;
		while(E<fsz+4)
			{
			path_AUDIOTRACK[(E/2)+2] = WFORMDYNAMIC[SPP+E];			//Fill path
			E=E+2;	
			}
		path_AUDIOTRACK[0] = 48;
		path_AUDIOTRACK[1] = 58;	
		fsz = WFORMDYNAMIC[StPosHead+8];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+9];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+10];
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+11];	
		StPosHead = StPosHead+fsz;	//PVBR position
		if(WFORMDYNAMIC[StPosHead] != 80 |
			 WFORMDYNAMIC[StPosHead+1] != 86 |
			 WFORMDYNAMIC[StPosHead+2] != 66 | 
			 WFORMDYNAMIC[StPosHead+3] != 82)	//Check PVBR position in file
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}	
		fsz = WFORMDYNAMIC[StPosHead+8];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+9];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+10];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+11];		
		StPosHead = StPosHead+fsz;	//PQTZ position
		if(WFORMDYNAMIC[StPosHead] != 80 |
			 WFORMDYNAMIC[StPosHead+1] != 81 |
			 WFORMDYNAMIC[StPosHead+2] != 84 | 
			 WFORMDYNAMIC[StPosHead+3] != 90)	//Check PQTZ position in file
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}	
		fsz = WFORMDYNAMIC[StPosHead+4];				
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+5];		
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+6];		
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+7];				//Head size
		uint32_t number_of_entries;
		number_of_entries = WFORMDYNAMIC[StPosHead+20];				
		number_of_entries = number_of_entries<<8;	
		number_of_entries+=WFORMDYNAMIC[StPosHead+21];		
		number_of_entries = number_of_entries<<8;
		number_of_entries+=WFORMDYNAMIC[StPosHead+22];		
		number_of_entries = number_of_entries<<8;
		number_of_entries+=WFORMDYNAMIC[StPosHead+23];				//calculate number_of_entries
		if(number_of_entries>4096)
			{
			number_of_entries = 4096;	
			}
		SPP = StPosHead + fsz + 2;						//start first BPM data.
		E = 0;
		GRID_OFFSET = WFORMDYNAMIC[SPP-1];							//find first beat 1...4
		while(E<number_of_entries)
			{
			BPMGRID[E] = WFORMDYNAMIC[SPP+(E*8)];	
			BPMGRID[E] = BPMGRID[E]<<8;
			BPMGRID[E]+= WFORMDYNAMIC[SPP+1+(E*8)];		
			BEATGRID[E] = WFORMDYNAMIC[SPP+2+(E*8)];	
			BEATGRID[E] = BEATGRID[E]<<8;
			BEATGRID[E]+= WFORMDYNAMIC[SPP+3+(E*8)];	
			BEATGRID[E] = BEATGRID[E]<<8;
			BEATGRID[E]+= WFORMDYNAMIC[SPP+4+(E*8)];	
			BEATGRID[E] = BEATGRID[E]<<8;
			BEATGRID[E]+= WFORMDYNAMIC[SPP+5+(E*8)];	
			BEATGRID[E] = (BEATGRID[E]*3)/20; 	//translate ms to 1/150s frames
			E++;	
			}
		if(E==4096)
			{
			BEATGRID[4095] = 0xFFFF;	
			}
		else
			{
			BEATGRID[E] = 0xFFFF;	
			BPMGRID[E] = BPMGRID[E-1];	
			}
		originalBPM = BPMGRID[0];			//SEND ORIGINAL BPM		

		fsz = WFORMDYNAMIC[StPosHead+8];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+9];
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+10];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+11];	
		StPosHead = StPosHead+fsz;	//PWAV position	
		if(WFORMDYNAMIC[StPosHead] != 80 |
			 WFORMDYNAMIC[StPosHead+1] != 87 |
			 WFORMDYNAMIC[StPosHead+2] != 65 | 
			 WFORMDYNAMIC[StPosHead+3] != 86)		//Check PWAV position in file
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}
		fsz = WFORMDYNAMIC[StPosHead+4];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+5];
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+6];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+7];				//PWAV Header size
		fsz+=StPosHead;		
		E = 0;
		while(E<400)
			{
			WFORMSTATIC[E] = WFORMDYNAMIC[fsz+E];			//Fill Static Waveform
			E++;	
			}		
		fsz = WFORMDYNAMIC[StPosHead+8];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+9];
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+10];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+11];				//PWAV TAG size
		StPosHead+=fsz;													//PWV2 start adress
		if(WFORMDYNAMIC[StPosHead] != 80 |
			 WFORMDYNAMIC[StPosHead+1] != 87 |
			 WFORMDYNAMIC[StPosHead+2] != 86 | 
			 WFORMDYNAMIC[StPosHead+3] != 50)		//Check PWV2 position in file
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}
		fsz = WFORMDYNAMIC[StPosHead+8];			
		fsz = fsz<<8;	
		fsz+=WFORMDYNAMIC[StPosHead+9];
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+10];	
		fsz = fsz<<8;
		fsz+=WFORMDYNAMIC[StPosHead+11];				//PWV2 Tag size
		StPosHead = StPosHead+fsz;	//PCOB position				
		if(WFORMDYNAMIC[StPosHead] != 80 |
			 WFORMDYNAMIC[StPosHead+1] != 67 |
			 WFORMDYNAMIC[StPosHead+2] != 79 | 
			 WFORMDYNAMIC[StPosHead+3] != 66)		//Check PCOB position in file
			{
			return 6;   //file ANLZXXXX.DAT is damadge!		
			}		
		DrawMemoryMarker(0, NONE_MARK, LCD_COLOR_BLACK);
		MemoryCuePyramid_ENABLE = 1;	
		uint32_t PCOB2_adr;
			
		//////////////////prepare HOT CUES	
		for(E=0;E<3;E++)									//Clear нcue massive
			{
			HCUE_type[E] = 0;	 							//b0 (0=cue / 1=loop); b1 (0=inactive / 1=active); 		
			HCUE_adr[0][E] = 0xFFFF;		
			HCUE_adr[1][E] = 0xFFFF;		
			}
		
		if(WFORMDYNAMIC[StPosHead+15]==1)    //check type PCOB - for HOT CUE points
			{
			number_of_hot_cue_points = WFORMDYNAMIC[StPosHead+19]&0xF;				//number of hotcue points		
			
			if(number_of_hot_cue_points>0)
				{
				MemoryCuePyramid_ENABLE = 3;	
				}
			fsz = WFORMDYNAMIC[StPosHead+8];			
			fsz = fsz<<8;	
			fsz+=WFORMDYNAMIC[StPosHead+9];
			fsz = fsz<<8;
			fsz+=WFORMDYNAMIC[StPosHead+10];	
			fsz = fsz<<8;
			fsz+=WFORMDYNAMIC[StPosHead+11];				//PCOB TAG size
			PCOB2_adr = fsz+StPosHead;							//start adress PCOB2
			fsz = WFORMDYNAMIC[StPosHead+4];			
			fsz = fsz<<8;	
			fsz+=WFORMDYNAMIC[StPosHead+5];
			fsz = fsz<<8;
			fsz+=WFORMDYNAMIC[StPosHead+6];	
			fsz = fsz<<8;
			fsz+=WFORMDYNAMIC[StPosHead+7];				//PCOB head size
			StPosHead+= fsz;		

			uint8_t HCUE_NAME = 0;
		
			E = 0;	
			while(E<number_of_hot_cue_points)	
				{	
				fsz = WFORMDYNAMIC[StPosHead+8];			
				fsz = fsz<<8;	
				fsz+=WFORMDYNAMIC[StPosHead+9];
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+10];	
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+11];				//PCPT TAG size
				SPP = StPosHead+fsz;										//Next PCPT adress

				HCUE_NAME = WFORMDYNAMIC[StPosHead+15];
				if(WFORMDYNAMIC[StPosHead+19]!=0)				//when hcue active
					{
					if((HCUE_NAME<4) && (HCUE_NAME>0))
						{
						HCUE_type[HCUE_NAME-1] = 2;			//write 
						}
					}
					
				fsz = WFORMDYNAMIC[StPosHead+4];			
				fsz = fsz<<8;	
				fsz+=WFORMDYNAMIC[StPosHead+5];
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+6];	
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+7];				//PCPT Head size
				StPosHead+= fsz;		
		
				if((HCUE_NAME<4) && (HCUE_NAME>0))
					{
					HCUE_NAME--;
					HCUE_adr[0][HCUE_NAME] = WFORMDYNAMIC[StPosHead+4];
					HCUE_adr[0][HCUE_NAME] = HCUE_adr[0][HCUE_NAME]<<8;
					HCUE_adr[0][HCUE_NAME]+= WFORMDYNAMIC[StPosHead+5];
					HCUE_adr[0][HCUE_NAME] = HCUE_adr[0][HCUE_NAME]<<8;
					HCUE_adr[0][HCUE_NAME]+= WFORMDYNAMIC[StPosHead+6];
					HCUE_adr[0][HCUE_NAME] = HCUE_adr[0][HCUE_NAME]<<8;	
					HCUE_adr[0][HCUE_NAME]+= WFORMDYNAMIC[StPosHead+7];
					if(WFORMDYNAMIC[StPosHead]==2)						//when hot cue type=loop
						{
						HCUE_type[HCUE_NAME] |= 0x1;		
						HCUE_adr[1][HCUE_NAME] = WFORMDYNAMIC[StPosHead+8];
						HCUE_adr[1][HCUE_NAME] = HCUE_adr[1][HCUE_NAME]<<8;
						HCUE_adr[1][HCUE_NAME]+= WFORMDYNAMIC[StPosHead+9];
						HCUE_adr[1][HCUE_NAME] = HCUE_adr[1][HCUE_NAME]<<8;
						HCUE_adr[1][HCUE_NAME]+= WFORMDYNAMIC[StPosHead+10];
						HCUE_adr[1][HCUE_NAME] = HCUE_adr[1][HCUE_NAME]<<8;	
						HCUE_adr[1][HCUE_NAME]+= WFORMDYNAMIC[StPosHead+11];
						HCUE_adr[1][HCUE_NAME] = (HCUE_adr[1][HCUE_NAME]*3)/20; 	//translate ms to 1/150s frames
						}
					}
				StPosHead = SPP;	
				E++;	
				}
			}
			
		//////////////////prepare MEMORY CUES
		for(E=0;E<10;E++)									//Clear memory cue massive
			{
			MEMORY_type[E] = 0;						//b0 (0=cue / 1=loop); b1 (0=inactive / 1=active); 		
			MEMORY_adr[0][E] = 0xFFFF;
			MEMORY_adr[1][E] = 0xFFFF;	
			}

		StPosHead = PCOB2_adr;	
		if(WFORMDYNAMIC[StPosHead+15]==0)    //check type PCOB - for MEMORY CUE points
			{
			number_of_memory_cue_points = WFORMDYNAMIC[StPosHead+19]&0xF;				//number of MEMORY points		
			if(number_of_memory_cue_points>0)
				{
				MemoryCuePyramid_ENABLE = 3;	
				}
			fsz = WFORMDYNAMIC[StPosHead+4];			
			fsz = fsz<<8;	
			fsz+=WFORMDYNAMIC[StPosHead+5];
			fsz = fsz<<8;
			fsz+=WFORMDYNAMIC[StPosHead+6];	
			fsz = fsz<<8;
			fsz+=WFORMDYNAMIC[StPosHead+7];				//PCOB head size
			StPosHead = StPosHead+fsz;		

			E = 0;	
			while(E<number_of_memory_cue_points)	
				{	
				fsz = WFORMDYNAMIC[StPosHead+8];			
				fsz = fsz<<8;	
				fsz+=WFORMDYNAMIC[StPosHead+9];
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+10];	
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+11];				//PCPT TAG size
				SPP = StPosHead+fsz;										//Next PCPT adress

				if(WFORMDYNAMIC[StPosHead+19]!=0)				//when hcue active
					{
					MEMORY_type[E] = 2;			//write 
					}	
					
				fsz = WFORMDYNAMIC[StPosHead+4];			
				fsz = fsz<<8;	
				fsz+=WFORMDYNAMIC[StPosHead+5];
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+6];	
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[StPosHead+7];				//PCPT Head size
				StPosHead = StPosHead+fsz;		
		
				MEMORY_adr[0][E] = WFORMDYNAMIC[StPosHead+4];
				MEMORY_adr[0][E] = MEMORY_adr[0][E]<<8;
				MEMORY_adr[0][E]+= WFORMDYNAMIC[StPosHead+5];
				MEMORY_adr[0][E] = MEMORY_adr[0][E]<<8;
				MEMORY_adr[0][E]+= WFORMDYNAMIC[StPosHead+6];
				MEMORY_adr[0][E] = MEMORY_adr[0][E]<<8;	
				MEMORY_adr[0][E]+= WFORMDYNAMIC[StPosHead+7];	
				if(WFORMDYNAMIC[StPosHead]==2)						//when hot cue type=loop
					{
					MEMORY_type[E] |= 0x1;		
					MEMORY_adr[1][E] = WFORMDYNAMIC[StPosHead+8];
					MEMORY_adr[1][E] = MEMORY_adr[1][E]<<8;
					MEMORY_adr[1][E]+= WFORMDYNAMIC[StPosHead+9];
					MEMORY_adr[1][E] = MEMORY_adr[1][E]<<8;
					MEMORY_adr[1][E]+= WFORMDYNAMIC[StPosHead+10];
					MEMORY_adr[1][E] = MEMORY_adr[1][E]<<8;	
					MEMORY_adr[1][E]+= WFORMDYNAMIC[StPosHead+11];	
					MEMORY_adr[1][E] = (MEMORY_adr[1][E]*3)/20; 	//translate ms to 1/150s frames
					}
				StPosHead = SPP;	
				E++;	
				}
			}
			
		path_ANLZ[42] = 69;								//	
		path_ANLZ[43] = 88;								//	Replace *.DAT to *.EXT
		path_ANLZ[44] = 84;								//	for open EXT file with dynamic waveform	

		res = f_open(&file, path_ANLZ, FA_READ);	
		if (res != FR_OK)
			{
			ERROR = 7;	//cannot open ANLZXXXX.EXT file
			}
		else
			{
			FILSIZE = f_size(&file);
			fsz = FILSIZE;
			if(fsz>135000)
				{
				fsz = 135000;	
				}
			res = f_read(&file, WFORMDYNAMIC, fsz, &nbytes);
			if(res != FR_OK)
				{
				ERROR = 8;	//ANLZXXXX.EXT file is damadge
				f_close(&file);					//Close file ANLZXXXX.EXT							
				}
			else
				{
				f_close(&file);					//Close file ANLZXXXX.EXT					
				fsz = WFORMDYNAMIC[8];			
				fsz = fsz<<8;	
				fsz+=WFORMDYNAMIC[9];	
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[10];	
				fsz = fsz<<8;
				fsz+=WFORMDYNAMIC[11];	//file size		
				if(fsz != FILSIZE)
					{
					ERROR = 9;	//ANLZXXXX.EXT file is damadge
					}
				else		
					{	
					fsz = WFORMDYNAMIC[4];			
					fsz = fsz<<8;	
					fsz+=WFORMDYNAMIC[5];	
					fsz = fsz<<8;
					fsz+=WFORMDYNAMIC[6];	
					fsz = fsz<<8;
					fsz+=WFORMDYNAMIC[7];	//Header size
					StPosHead = fsz;	
					if(WFORMDYNAMIC[StPosHead] != 80 | 
						 WFORMDYNAMIC[StPosHead+1] != 80 | 
					   WFORMDYNAMIC[StPosHead+2] != 84 | 
					   WFORMDYNAMIC[StPosHead+3] != 72)		//Check PPTH position in file
						{
						ERROR = 10;	//ANLZXXXX.EXT file is damadge				
						}
					else
						{		
						fsz = WFORMDYNAMIC[StPosHead+8];			
						fsz = fsz<<8;	
						fsz+=WFORMDYNAMIC[StPosHead+9];	
						fsz = fsz<<8;
						fsz+=WFORMDYNAMIC[StPosHead+10];	
						fsz = fsz<<8;
						fsz+=WFORMDYNAMIC[StPosHead+11];	//Tag size
						StPosHead += fsz;		
						if(WFORMDYNAMIC[StPosHead] != 80 | 
							 WFORMDYNAMIC[StPosHead+1] != 87 | 
						   WFORMDYNAMIC[StPosHead+2] != 86 | 
						   WFORMDYNAMIC[StPosHead+3] != 51)		//Check PWV3 position in file
							{
							return 11;	//ANLZXXXX.EXT file is damadge			
							}
						else
							{	
							fsz = WFORMDYNAMIC[StPosHead+4];			
							fsz = fsz<<8;	
							fsz+=WFORMDYNAMIC[StPosHead+5];	
							fsz = fsz<<8;
							fsz+=WFORMDYNAMIC[StPosHead+6];	
							fsz = fsz<<8;
							fsz+=WFORMDYNAMIC[StPosHead+7];	//Header size
							SPP = StPosHead + fsz; //Start position waveform	 
									
							fsz = WFORMDYNAMIC[StPosHead+16];			
							fsz = fsz<<8;	
							fsz+=WFORMDYNAMIC[StPosHead+17];	
							fsz = fsz<<8;
							fsz+=WFORMDYNAMIC[StPosHead+18];	
							fsz = fsz<<8;
							fsz+=WFORMDYNAMIC[StPosHead+19];		//waveform data size
							if(fsz>1)				//delete 2 end frames (exclude noise) 
								{
								fsz-=2;	
								}
							else
								{
								fsz = 0;	
								}									
							all_long = fsz;
							
							if(fsz>135000-SPP)
								{
								fsz = 135000-SPP;	
								}

							uint32_t j;							//Data shift
							for(j=0;j<fsz;j++)
								{
								WFORMDYNAMIC[j] = WFORMDYNAMIC[j+SPP];	
								}
								
							for(E=0;E<number_of_memory_cue_points;E++)											//Draw CUES on Display
								{
								if(MEMORY_adr[0][E] != 0xFFFF)
									{
									DrawMemoryMarker(1+(60*MEMORY_adr[0][E]/all_long), MEMORY_MARK, LCD_COLOR_RED);	
									MEMORY_adr[0][E] = (MEMORY_adr[0][E]*3)/20;				//translate ms to 1/150s frames
									}	
								}	

							////////////////////////////////////sorting algoritm	
							uint8_t MINM, ii;
												
							for(E=0;E<number_of_memory_cue_points-1;E++)	
								{
								MINM = E;
								for(ii=(E+1);ii<number_of_memory_cue_points;ii++)
									{
									if(MEMORY_adr[0][ii]<MEMORY_adr[0][MINM])
										{
										j = MEMORY_adr[0][MINM];
										MEMORY_adr[0][MINM] = MEMORY_adr[0][ii];
										MEMORY_adr[0][ii] = j;
										j = MEMORY_adr[1][MINM];
										MEMORY_adr[1][MINM] = MEMORY_adr[1][ii];
										MEMORY_adr[1][ii] = j;												
										j = MEMORY_type[MINM];
										MEMORY_type[MINM] = MEMORY_type[ii];
										MEMORY_type[ii] = j;	
										}
									}
								}

							for(E=0;E<3;E++)											//Draw MEMORY on Display
								{
								if(HCUE_adr[0][E] != 0xFFFF)
									{
									if(HCUE_type[E]&0x1)
										{
										DrawMemoryMarker(1+(60*HCUE_adr[0][E]/all_long), 2+E, CUE_COLOR);				//Draw ORANGE MARKER
										}
									else
										{
										DrawMemoryMarker(1+(60*HCUE_adr[0][E]/all_long), 2+E, LCD_COLOR_GREEN);				//Draw GREEN MARKER	
										}
									HCUE_adr[0][E] = (HCUE_adr[0][E]*3)/20;				//translate ms to 1/150s frames										
									if(E==0)							//HCUE_A
										{
										if(HCUE_type[E]&0x1)
											{
											Tbuffer[18] |= 0x1;	
											Tbuffer[17] &= 0x7F;	
											}
										else
											{
											Tbuffer[17] |= 0x80;
											Tbuffer[18] &= 0x7E;	
											}
										}
									else if(E==1)					//HCUE_B
										{
										if(HCUE_type[E]&0x1)
											{
											Tbuffer[18] |= 0x8;	
											Tbuffer[18] &= 0x7B;	
											}
										else
											{	
											Tbuffer[18] |= 0x4;
											Tbuffer[18] &= 0x77;	
											}
										}	
									else if(E==2)					//HCUE_C
										{
										if(HCUE_type[E]&0x1)
											{
											Tbuffer[18] |= 0x40;		
											Tbuffer[18] &= 0x5F;	
											}
										else
											{	
											Tbuffer[18] |= 0x20;
											Tbuffer[18] &= 0x3F;	
											}
										}	
									}
								else				//turn off leds
									{
									if(E==0)							//HCUE_A
										{
										Tbuffer[17] &= 0x7F;
										Tbuffer[18] &= 0x7E;	
										}
									else if(E==1)					//HCUE_B
										{
										Tbuffer[18] &= 0x73;		 
										}	
									else if(E==2)					//HCUE_C
										{	
										Tbuffer[18] &= 0x1F;		
										}
									}									
								}
							}
						}
					}			
				}	
			}

		res = f_open(&file, path_AUDIOTRACK, FA_READ);				//Open audio file
		if (res != FR_OK)
			{
			return 13;	//cannot open AUDIOTRACK
			}
		res = f_read(&file, PCM, 512, &nbytes);
		if(res != FR_OK)
			{
			ERROR = 14;	//cannot read AUDIOTRACK
 			}
		if(PCM[0][5][0] != 1 || PCM[0][5][1] != 2 || PCM[0][6][0] != 44100 || PCM[0][8][1] != 16)	//Check audio format
			{
			ERROR = 15;	//unsupported audio format	
			}
		f_lseek(&file, 44);
		if(ERROR==0)
			{
			playlist[TRACK_NUMBER-1][54] |= 0x01;				//write history mem
			}
		return ERROR;	
		}

		
///////////////////////////////////////////		
//preparing for loading next track
//
void PREPARE_LOAD_TRACK(uint16_t TRACK_NUMBER, uint16_t TRACK_IN_PLAYLIST)
	{
	lock_control = 1;	
	uint16_t ERR = 0;	
	f_close(&file);	
	pitch = 0;	
	play_enable = 0;
	if(Tbuffer[19]&0x8)					//OFF_SLIP_MODE
		{
		Tbuffer[19] &= 0xF7;	
		}		
	play_adr = 0;	
	slip_play_adr = 0;
	loop_active = 0;
	LOOP_OUT = 0;
			
	ERR = LOAD_TRACK(TRACK_NUMBER);		
	if(ERR==0)								//the sequence of functions in this place is very important!
		{		
		tempo_need_update = 1;		
		end_adr_valid_data = 0;
		start_adr_valid_data = 0;	
		RED_CIRCLE_CUE_ADR = 0;	
		if(QUANTIZE)
			{
			ShowQUANTIZE(2);	
			}	
		ShowTrackNumber(TRACK_IN_PLAYLIST);
		DrawMemoryCuePyramid(MemoryCuePyramid_ENABLE);	
		if(UTILITY_SETTINGS[2]==9)					//first beat
			{
			play_adr = BEATGRID[0]*294;	
			slip_play_adr = play_adr;
			CUE_ADR = BEATGRID[0];	
			}
		else if(UTILITY_SETTINGS[2]==8 && number_of_memory_cue_points>0)					//memory
			{	
			SET_MEMORY_CUE_1(MEMORY_adr[0][0]);
			CUE_OPERATION = MEMORY_NEED_SET_PART2;
			CUE_ADR =	MEMORY_adr[0][0];
			}	
		else
			{
			uint8_t have_a_cue = 0;
			uint16_t c = 0;
			uint16_t M = 0;	
			while(have_a_cue==0 && all_long>28*c)
				{
				f_read(&file, PCM[0][0], 32768, &nbytes);	
				for(M=0;M<8192;M++)
					{
					if(PCM[0][M][0]&0x8000)		//negative 65535...32768
						{
						PCM[0][M][0] = 0xFFFF - PCM[0][M][0];	
						}
					if(((PCM[0][M][0]>>(9-UTILITY_SETTINGS[2]))>0) && have_a_cue==0)
						{
						have_a_cue = 1;
						CUE_ADR = (c*8192+M)/294;
						play_adr = CUE_ADR*294;
						slip_play_adr = play_adr;
						M = 0xFFFF;	
						}
					if(PCM[0][M][1]&0x8000)		//negative 65535...32768
						{
						PCM[0][M][1] = 0xFFFF - PCM[0][M][1];	
						}
					if(((PCM[0][M][1]>>(9-UTILITY_SETTINGS[2]))>0) && have_a_cue==0)
						{
						have_a_cue = 1;
						CUE_ADR = (c*8192+M)/294;
						play_adr = CUE_ADR*294;
						slip_play_adr = play_adr;
						M = 0xFFFF;	
						}
					}	
				}
			f_lseek(&file, 44);						//return to start track	
				
			if(have_a_cue==0)
				{
				play_adr = 0;	
				slip_play_adr = 0;	
				CUE_ADR = 0;		
				}		
			}			
			
		DrawStaticWFM(DRAW_NEW_STATIC_WAVEFORM);				//Draw New waveform		
	
		if(UTILITY_SETTINGS[2]==9)					//first beat
			{
			RedrawWaveforms(BEATGRID[0]);
			}	
		else if(UTILITY_SETTINGS[2]==8 && number_of_memory_cue_points>0)					//memory
			{	
			RedrawWaveforms(MEMORY_adr[0][0]);	
			}			
		else		
			{
			RedrawWaveforms(CUE_ADR);	
			}
		
		SwitchInformationLayer(WAVEFORM);	

		if(UTILITY_SETTINGS[2]==9)					//first beat
			{	
			SET_CUE(BEATGRID[0]);						//AUTO CUE set
			}
		else if(UTILITY_SETTINGS[2]==8 && number_of_memory_cue_points>0)					//memory
			{	
			SET_CUE(MEMORY_adr[0][0]);
			}			
		else
			{
			SET_CUE(CUE_ADR);						//AUTO CUE set	
			}	
			
		DrawMinuteMarkers();			
		}
	else
		{
		all_long = 0;		
		originalBPM = 0xFFFF;	
		ShowBPM(originalBPM);
		if(QUANTIZE)
			{
			ShowQUANTIZE(1);	
			}	
		else
			{
			ShowQUANTIZE(0);		
			}
		ShowTrackNumber(0);
		MemoryCuePyramid_ENABLE = 0;	
		DrawMemoryCuePyramid(MemoryCuePyramid_ENABLE);
		SwitchInformationLayer(WAVEFORM);	
		DrawStaticWFM(CLEAR_WAVEFORM_ARRAY);	
		DrawStaticWFM(MS_ERROR+ERR);	
		RED_CIRCLE_CUE_ADR = 0;	
		}

	/////////////////////////////////////////////////////////////////////////////////////write utility settigs to flash, when track stopped	
	if(need_rewrite_flash && dSHOW!=UTILITY)																																			
		{																																																							
		HAL_FLASH_Unlock();																																														
		FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_VOLTAGE_RANGE_3);					//clear flash															
		FLASH_WaitForLastOperation(50000); 																																						
		CLEAR_BIT(FLASH->CR, (FLASH_CR_SER | FLASH_CR_SNB));																													
		for(U=0;U<13;U++)								///////////write																															
			{																																																						
			if(FLASH_WaitForLastOperation(50000) != HAL_TIMEOUT)																												
				{																																																					
				__HAL_FLASH_CLEAR_FLAG(	FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |										
										FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_SR_ERSERR);																			
				}																																																					
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, UTILITY_START_ADDR+U, (uint32_t)UTILITY_SETTINGS[U]);							
			}																																																						
		HAL_FLASH_Lock();																																															
		need_rewrite_flash = 0;																																												
		}	
		
	if(ERR==0)								//the sequence of functions in this place is very important!
		{		
		lock_control = 0;
		}
	return;	
	}	


//////////////////////////////////////	
//	Show Phase meter 0 - none 1...4 phase	
//		
void ShowPhaseMeter(uint16_t phase)
	{
	uint8_t j;	
	uint16_t int_offset;
		
	if(PreviousPhase==phase)
		{
		return;	
		}
	if(phase==0xFFFF)
		{
		BSP_LCD_SetFont(&Font13D);	
		BSP_LCD_SetTextColor(CUE_COLOR);
		BSP_LCD_DrawRect(178, 36, 29, 9);		
		BSP_LCD_FillRect(207, 39, 3, 3);	
		BSP_LCD_DrawRect(210, 36, 29, 9);
		BSP_LCD_FillRect(239, 39, 3, 3);
		BSP_LCD_DrawRect(242, 36, 29, 9);
		BSP_LCD_FillRect(271, 39, 3, 3);	
		BSP_LCD_DrawRect(274, 36, 29, 9);
		sprintf((char*)Buf, "%s", ".");	
		BSP_LCD_DisplayStringAt(327, 33, Buf, LEFT_MODE);	
		sprintf((char*)Buf, "%s", "-");	
		BSP_LCD_DisplayStringAt(310, 33, Buf, LEFT_MODE);
		sprintf((char*)Buf, "%s", "-");	
		BSP_LCD_DisplayStringAt(321, 33, Buf, LEFT_MODE);		
		sprintf((char*)Buf, "%s", "-");				
		BSP_LCD_DisplayStringAt(336, 33, Buf, LEFT_MODE);		
		BSP_LCD_SetFont(&FontBMP);
		sprintf((char*)Buf, "%s", ":");						//Bars
		BSP_LCD_DisplayStringAt(348, 40, Buf, TRANSPARENT_MODE);
		sprintf((char*)Buf, "%s", ";<=");						//MASTER PLAYER
		BSP_LCD_DisplayStringAt(117, 29, Buf, TRANSPARENT_MODE);
		sprintf((char*)Buf, "%s", ">?@");						//MASTER PLAYER
		BSP_LCD_DisplayStringAt(117, 40, Buf, TRANSPARENT_MODE);
		BSP_LCD_SetTextColor(0x55F08138);						//GRADIENT
		BSP_LCD_DrawHLine(179, 40, 27);	
		BSP_LCD_DrawHLine(211, 40, 27);	
		BSP_LCD_DrawHLine(243, 40, 27);	
		BSP_LCD_DrawHLine(275, 40, 27);	
		BSP_LCD_SetTextColor(0x69F08138);
		BSP_LCD_DrawHLine(179, 41, 27);	
		BSP_LCD_DrawHLine(211, 41, 27);	
		BSP_LCD_DrawHLine(243, 41, 27);	
		BSP_LCD_DrawHLine(275, 41, 27);	
		BSP_LCD_SetTextColor(0x82F08138);
		BSP_LCD_DrawHLine(179, 42, 27);	
		BSP_LCD_DrawHLine(211, 42, 27);	
		BSP_LCD_DrawHLine(243, 42, 27);	
		BSP_LCD_DrawHLine(275, 42, 27);	
		BSP_LCD_SetTextColor(0x9BF08138);
		BSP_LCD_DrawHLine(179, 43, 27);	
		BSP_LCD_DrawHLine(211, 43, 27);	
		BSP_LCD_DrawHLine(243, 43, 27);	
		BSP_LCD_DrawHLine(275, 43, 27);	
		BSP_LCD_SetFont(&Font13D);	
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_DrawRect(178, 50, 29, 9);
		BSP_LCD_FillRect(207, 53, 3, 3);	
		BSP_LCD_DrawRect(210, 50, 29, 9);
		BSP_LCD_FillRect(239, 53, 3, 3);
		BSP_LCD_DrawRect(242, 50, 29, 9);
		BSP_LCD_FillRect(271, 53, 3, 3);	
		BSP_LCD_DrawRect(274, 50, 29,	 9);	
		sprintf((char*)Buf, "%s", ".");	
		BSP_LCD_DisplayStringAt(327,49, Buf, LEFT_MODE);	
		sprintf((char*)Buf, "%s", "-");	
		BSP_LCD_DisplayStringAt(310, 49, Buf, LEFT_MODE);
		sprintf((char*)Buf, "%s", "-");	
		BSP_LCD_DisplayStringAt(321, 49, Buf, LEFT_MODE);		
		sprintf((char*)Buf, "%s", "-");				
		BSP_LCD_DisplayStringAt(336, 49, Buf, LEFT_MODE);			
		BSP_LCD_SetFont(&FontBMP);
		sprintf((char*)Buf, "%s", ":");						//Bars
		BSP_LCD_DisplayStringAt(348, 56, Buf, TRANSPARENT_MODE);
		BSP_LCD_SetTextColor(0x558080FF);					//GRADIENT
		BSP_LCD_DrawHLine(179, 54, 27);	
		BSP_LCD_DrawHLine(211, 54, 27);	
		BSP_LCD_DrawHLine(243, 54, 27);	
		BSP_LCD_DrawHLine(275, 54, 27);	
		BSP_LCD_SetTextColor(0x698080FF);
		BSP_LCD_DrawHLine(179, 53, 27);	
		BSP_LCD_DrawHLine(211, 53, 27);	
		BSP_LCD_DrawHLine(243, 53, 27);	
		BSP_LCD_DrawHLine(275, 53, 27);	
		BSP_LCD_SetTextColor(0x828080FF);
		BSP_LCD_DrawHLine(179, 52, 27);	
		BSP_LCD_DrawHLine(211, 52, 27);	
		BSP_LCD_DrawHLine(243, 52, 27);	
		BSP_LCD_DrawHLine(275, 52, 27);	
		BSP_LCD_SetTextColor(0x9B8080FF);
		BSP_LCD_DrawHLine(179, 51, 27);	
		BSP_LCD_DrawHLine(211, 51, 27);	
		BSP_LCD_DrawHLine(243, 51, 27);	
		BSP_LCD_DrawHLine(275, 51, 27);	
		}
	else
		{
		int_offset = ((2+GRID_OFFSET)&0x0003);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
		BSP_LCD_FillRect((179+32*((PreviousPhase+int_offset)%4)), 55, 27, 3);
		BSP_LCD_SetTextColor(0x558080FF);
		BSP_LCD_DrawHLine((179+32*((PreviousPhase+int_offset)%4)), 54, 27);		
		BSP_LCD_SetTextColor(0x698080FF);
		BSP_LCD_DrawHLine((179+32*((PreviousPhase+int_offset)%4)), 53, 27);	
		BSP_LCD_SetTextColor(0x828080FF);
		BSP_LCD_DrawHLine((179+32*((PreviousPhase+int_offset)%4)), 52, 27);	
		BSP_LCD_SetTextColor(0x9B8080FF);
		BSP_LCD_DrawHLine((179+32*((PreviousPhase+int_offset)%4)), 51, 27);	
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);	
		BSP_LCD_DrawRect(178+32*((PreviousPhase+int_offset)%4), 50, 29, 9);	
			
		int_offset+= phase;	
		if(int_offset%4==0)									//RED PHASE
			{
			for(j=0;j<56;j++)	
				{
				BSP_LCD_DrawPixel((192+j%14), 54-(j/14), RED_BAR[j/14][j%14]);	
				BSP_LCD_DrawPixel((192+j%14), 54+(j/14), RED_BAR[j/14][j%14]);		
				BSP_LCD_DrawPixel((192-j%14), 54-(j/14), RED_BAR[j/14][j%14]);	
				BSP_LCD_DrawPixel((192-j%14), 54+(j/14), RED_BAR[j/14][j%14]);		
				}	
			}
		else																						//BLUE PHASE
			{	
			for(j=0;j<56;j++)	
				{
				BSP_LCD_DrawPixel((192+j%14)+32*(int_offset%4), 54-(j/14), BLUE_BAR[j/14][j%14]);	
				BSP_LCD_DrawPixel((192+j%14)+32*(int_offset%4), 54+(j/14), BLUE_BAR[j/14][j%14]);		
				BSP_LCD_DrawPixel((192-j%14)+32*(int_offset%4), 54-(j/14), BLUE_BAR[j/14][j%14]);	
				BSP_LCD_DrawPixel((192-j%14)+32*(int_offset%4), 54+(j/14), BLUE_BAR[j/14][j%14]);		
				}	
			}	
		BSP_LCD_SetTextColor(0xFFA6C8FF);	
		BSP_LCD_DrawRect(178+32*(int_offset%4), 50, 29, 9);		

		BSP_LCD_SetFont(&Font13D);
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
		sprintf((char*)Buf, "%s", ".");	
		BSP_LCD_DisplayStringAt(327, 49, Buf, LEFT_MODE);	
		sprintf((char *)Buf , "%01lu", (int_offset/40)%10);	
		BSP_LCD_DisplayStringAt(310, 49, Buf, LEFT_MODE);
		sprintf((char *)Buf, "%01lu", (int_offset>>2)%10);	
		BSP_LCD_DisplayStringAt(321, 49, Buf, LEFT_MODE);		
		sprintf((char *)Buf , "%01lu", (int_offset%4)+1);					
		BSP_LCD_DisplayStringAt(336, 49, Buf, LEFT_MODE);	
		}		
	PreviousPhase = phase;
	return;	
	}

//Switch Dynamic Waveform/Browser
//
//	
void SwitchInformationLayer(uint8_t LAY)
	{
	uint8_t update_all_page = 0;
	if(dSHOW != LAY)
		{
		dSHOW = LAY;
		update_all_page = 1;	
		}	
	uint16_t E;	
	if(LAY==WAVEFORM)		//X => WAVEFORM
		{
		HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_RESET);		
		BSP_LCD_SelectLayer(1);
		BSP_LCD_SetTransparency(1, 0);	
		BSP_LCD_Clear(0x00000000);
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
		BSP_LCD_FillRect(0, 0, 480, 182);	
		int_DRAW_TRANSPARENT_BAR();
		BSP_LCD_SetFont(&Font15P);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		if(track_play_now>0)
			{	
			sprintf((char*)Buf, "%s", "~");
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);		
			BSP_LCD_DisplayStringAt(21,2, playlist[track_play_now-1], WAVEFORM_MODE);
				
			for(E=0;E<224;E++)					////KEY ICON
				{
				if((iTONE[E/16]>>(E%16))%2)
					{
					BSP_LCD_DrawPixel(433-E%16, 2+E/16,  COLOR_MAP_RATING[6]);
					}	
				}	
			BSP_LCD_SetTextColor(COLOR_MAP_RATING[6]);	
			int_VALUE_to_KEY(key_id[track_play_now-1]);	
			BSP_LCD_DisplayStringAt(436, 2,Buf, TRANSPARENT_MODE);		
			}
		DrawMemoryCuePyramid(MemoryCuePyramid_ENABLE);
		DrawZOOMGRID();
		PreviousPhase = 0;										//forcibly draw phasemeter static parts
		ShowPhaseMeter(0xFFFF);		
		if(track_play_now!=0)
			{	
			forcibly_redraw = 1;	
			}
		else
			{
			DrawREKORDBOX();	
			}
		return;
		}
	else if(LAY==BROWSER)													//X => BROWSER
		{
		if(update_all_page==1)
			{
			HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_RESET);		
			BSP_LCD_SetTransparency(1, 0);	
			BSP_LCD_SelectLayer(1);
			BSP_LCD_Clear(0x00000000);
			BSP_LCD_SelectLayer(0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			BSP_LCD_FillRect(0, 0, 480, 182);	
			int_DRAW_TRANSPARENT_BAR();
				
			intDrawLayer0_NOINFO(B0CurrentCursorPosition);	
			
			BSP_LCD_SelectLayer(1);	
			BSP_LCD_SetTransparency(1, 255);		
			BSP_LCD_SetFont(&Font15P);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			sprintf((char*)Buf, "%s", "|");		
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);				
			sprintf((char*)Buf, "%s", TRACKLIST_NAME[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]);						
			BSP_LCD_DisplayStringAt(23, 2, Buf, TRANSPARENT_MODE);
			sprintf((char*)Buf, "%s", "Total Track");						
			BSP_LCD_DisplayStringAt(350,2,Buf, TRANSPARENT_MODE);
			sprintf((char *)Buf , "%1lu", TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition] - TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]);					
			BSP_LCD_DisplayStringAt(448,2,Buf, TRANSPARENT_MODE);		
			}
		else
			{
			BSP_LCD_SelectLayer(1);		
			BSP_LCD_SetTextColor(0x00000000);	
			BSP_LCD_FillRect(50, 20+(19*B0CurrentCursorPosition), 15, 19);		
			}
		int_B_DRAW_ALL_LINES();	
		return;
		}
	else if(LAY==TAG_LIST)													//X => TAG LIST
		{
		HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_RESET);		
		BSP_LCD_SetTransparency(1, 0);	
		BSP_LCD_SelectLayer(1);
		BSP_LCD_Clear(0x00000000);	
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
		BSP_LCD_FillRect(0, 0, 480, 182);	
			
		BSP_LCD_SetTextColor(0xFF173367);
		BSP_LCD_FillRect(0, 0, 480, 18);		
			
		intDrawLayer0_NOINFO(TCurrentCursorPosition);
			
		BSP_LCD_SelectLayer(1);	
		BSP_LCD_SetTransparency(1, 255);		
		BSP_LCD_SetFont(&Font15P);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		sprintf((char*)Buf, "%s", "TAG LIST");			
		BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
		sprintf((char*)Buf, "%s", "Total Track");						
		BSP_LCD_DisplayStringAt(350,2,Buf, TRANSPARENT_MODE);
		sprintf((char *)Buf , "%01lu", TOTAL_TRACKS_IN_TAG_LIST);					
		BSP_LCD_DisplayStringAt(448,2,Buf, TRANSPARENT_MODE);	
		int_T_DRAW_ALL_LINES();	
		return;	
		}
	else if(LAY==BROWSER_INFO)													//X => BROWSER_INFO
		{
		if(BROWSE_LEVEL==0)	
			{
			if(update_all_page==1)
				{	
				HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_SET);	
				HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_RESET);				
				BSP_LCD_SetTransparency(1, 0);	
				BSP_LCD_SelectLayer(1);
				BSP_LCD_Clear(0x00000000);	
				BSP_LCD_SelectLayer(0);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
				BSP_LCD_FillRect(0, 0, 480, 182);		
				int_DRAW_TRANSPARENT_BAR();	
			
				intDrawLayer0_INFO(B0CurrentCursorPosition);
								
				BSP_LCD_SelectLayer(1);	
				BSP_LCD_SetTransparency(1, 255);		
				BSP_LCD_SetFont(&Font15P);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);					
				sprintf((char*)Buf, "%s", "|");		
				BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);				
				sprintf((char*)Buf, "%s", TRACKLIST_NAME[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]);						
				BSP_LCD_DisplayStringAt(23, 2, Buf, TRANSPARENT_MODE);
				sprintf((char*)Buf, "%s", "Total Track");						
				BSP_LCD_DisplayStringAt(350,2,Buf, TRANSPARENT_MODE);
				sprintf((char *)Buf , "%1lu", TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition] - TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]);					
				BSP_LCD_DisplayStringAt(448,2,Buf, TRANSPARENT_MODE);
				}
			else
				{
				BSP_LCD_SelectLayer(1);		
				BSP_LCD_SetTextColor(0x00000000);	
				BSP_LCD_FillRect(20, 20+(19*B0CurrentCursorPosition), 15, 19);		
				}
			int_BIx_DRAW_ALL_LINES(0);	
			}
		else		
			{
			HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_RESET);				
			BSP_LCD_SetTransparency(1, 0);	
			BSP_LCD_SelectLayer(1);
			BSP_LCD_Clear(0x00000000);	
			BSP_LCD_SelectLayer(0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			BSP_LCD_FillRect(0, 0, 480, 182);		
			int_DRAW_TRANSPARENT_BAR();
			
			if(BROWSE_LEVEL==1)	
				{
				intDrawLayer0_BROWSER_1_3(B1CurrentCursorPosition);	
				BSP_LCD_SelectLayer(1);	
				BSP_LCD_SetTransparency(1, 255);		
				BSP_LCD_SetFont(&Font15P);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				sprintf((char*)Buf, "%s", "[PLAYLIST]");						
				BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
				}
			else if(BROWSE_LEVEL==2)	
				{
				intDrawLayer0_BROWSER_1_3(B2CurrentCursorPosition);	
				BSP_LCD_SelectLayer(1);	
				BSP_LCD_SetTransparency(1, 255);		
				BSP_LCD_SetFont(&Font15P);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				sprintf((char*)Buf, "%s", "/");		
				BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);				
				sprintf((char*)Buf, "%s", "SD card");						
				BSP_LCD_DisplayStringAt(23, 2, Buf, TRANSPARENT_MODE);					
				}
			else if(BROWSE_LEVEL==3)	
				{
				intDrawLayer0_BROWSER_1_3(0);
				BSP_LCD_SelectLayer(1);	
				BSP_LCD_SetTransparency(1, 255);		
				BSP_LCD_SetFont(&Font15P);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				sprintf((char*)Buf, "%s", "SD card slot");						
				BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
				}	
			int_BIx_DRAW_ALL_LINES(BROWSE_LEVEL);	
			}
		return;
		}
	else if(LAY==TAG_LIST_INFO)													//X => TAG LIST INFO
		{
		HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_RESET);				
		BSP_LCD_SetTransparency(1, 0);	
		BSP_LCD_SelectLayer(1);
		BSP_LCD_Clear(0x00000000);	
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
		BSP_LCD_FillRect(0, 0, 480, 182);	
			
		BSP_LCD_SetTextColor(0xFF173367);
		BSP_LCD_FillRect(0, 0, 480, 18);		
		
		intDrawLayer0_INFO(TCurrentCursorPosition);

		BSP_LCD_SelectLayer(1);	
		BSP_LCD_SetTransparency(1, 255);		
		BSP_LCD_SetFont(&Font15P);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		sprintf((char*)Buf, "%s", "TAG LIST");			
		BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
		sprintf((char*)Buf, "%s", "Total Track");						
		BSP_LCD_DisplayStringAt(350,2,Buf, TRANSPARENT_MODE);
		sprintf((char *)Buf , "%01lu", TOTAL_TRACKS_IN_TAG_LIST);					
		BSP_LCD_DisplayStringAt(448,2,Buf, TRANSPARENT_MODE);	
		int_TI_DRAW_ALL_LINES();		
		return;	
		}
	else if(LAY==UTILITY)															//X => UTILITY
		{
		if(update_all_page)
			{
			HAL_GPIO_WritePin(GPIOB, LED_TAG_LIST_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_BROWSE_Pin, GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);		
			HAL_GPIO_WritePin(GPIOB, LED_MENU_Pin, GPIO_PIN_SET);	
			}	
		BSP_LCD_SetTransparency(1, 0);	
		BSP_LCD_SelectLayer(1);	
		if(update_all_page)
			{
			BSP_LCD_Clear(0x00000000);	
			}	
		BSP_LCD_SelectLayer(0);
		if(update_all_page)	
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			BSP_LCD_FillRect(0, 0, 480, 182);	
			BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
			BSP_LCD_FillRect(0, 0, 480, 18);		
			BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
			BSP_LCD_DrawRect(0, 18, 14, 152);	
			BSP_LCD_SetTextColor(0xFF0F0F0);	
			BSP_LCD_FillRect(14, 18, 466, 152);	
			BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
			BSP_LCD_DrawLine(14,18,479,18);
			BSP_LCD_DrawLine(14,37,479,37);
			BSP_LCD_DrawLine(14,56,479,56);
			BSP_LCD_DrawLine(14,75,479,75);
			BSP_LCD_DrawLine(14,94,479,94);
			BSP_LCD_DrawLine(14,113,479,113);
			BSP_LCD_DrawLine(14,132,479,132);
			BSP_LCD_DrawLine(14,151,479,151);
			BSP_LCD_DrawLine(0,170,479,170);
			BSP_LCD_DrawLine(479,18,479,170);
			BSP_LCD_DrawLine(270,18,270,170);				//vertical line
			}
		if(edit_parameter==0)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
			BSP_LCD_FillRect(14, (18+(19*UCurrentCursorPosition)), 256, 9);			////Draw selected cursor	
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
			BSP_LCD_FillRect(14, (27+(19*UCurrentCursorPosition)), 256, 5);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
			BSP_LCD_FillRect(14, (32+(19*UCurrentCursorPosition)), 256, 5);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_3);		
			BSP_LCD_FillRect(271, (18+(19*UCurrentCursorPosition)), 208, 9);			////Draw selected cursor	
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_5);
			BSP_LCD_FillRect(271, (27+(19*UCurrentCursorPosition)), 208, 5);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_4);
			BSP_LCD_FillRect(271, (32+(19*UCurrentCursorPosition)), 208, 5);				
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			FillTriangle(20, 20, 27, 20+(19*UCurrentCursorPosition), 34+(19*UCurrentCursorPosition), 27 +(19*UCurrentCursorPosition));	
			}
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_3);		
			BSP_LCD_FillRect(14, (18+(19*UCurrentCursorPosition)), 256, 9);			////Draw selected cursor	
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_5);
			BSP_LCD_FillRect(14, (27+(19*UCurrentCursorPosition)), 256, 5);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_4);
			BSP_LCD_FillRect(14, (32+(19*UCurrentCursorPosition)), 256, 5);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);				
			BSP_LCD_FillRect(271, (18+(19*UCurrentCursorPosition)), 208, 9);			////Draw selected cursor	
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
			BSP_LCD_FillRect(271, (27+(19*UCurrentCursorPosition)), 208, 5);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
			BSP_LCD_FillRect(271, (32+(19*UCurrentCursorPosition)), 208, 5);				
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			FillTriangle(276, 276, 283, 20+(19*UCurrentCursorPosition), 34+(19*UCurrentCursorPosition), 27 +(19*UCurrentCursorPosition));	
			}		
		BSP_LCD_SelectLayer(1);	
		BSP_LCD_SetTransparency(1, 255);
		BSP_LCD_SetFont(&Font15P);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		if(update_all_page)
			{	
			sprintf((char*)Buf, "%s", "UTILITY");			
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
			}
		int_U_DRAW_ALL_LINES();	
		return;	
		}
	else if(LAY==BROWSER_NAVI)		//X => BROWSER NAVIGATION WITH ANIMATION
		{
		CURRENT_LAY = ActiveLayer;		
		BSP_LCD_SetTransparency(1, 0);	
		if(ActiveLayer !=1)
			{
			BSP_LCD_SelectLayer(1);
			}	
		BSP_LCD_Clear(0x00000000);
			
		if(PREVIOUS_BROWSE_LEVEL<BROWSE_LEVEL)				//reverse
			{
			if(BROWSE_LEVEL==1 && BROWSER_INFO_enable==0)	
				{
				info_animation_enable = 0;			
				}
			else
				{
				BSP_LCD_SetTextColor(LCD_COLOR_PAPER_TRANSP);		
				BSP_LCD_FillRect(270, 18, 210, 152);		
				info_animation_enable = 1;
				}
			animation_en = 2;		
			}
		else																				//forward
			{
			if(BROWSE_LEVEL==0 && BROWSER_INFO_enable==0)	
				{
				info_animation_enable = 0;		
				}
			else
				{	
				info_animation_enable = 1;		
				}
			BSP_LCD_SetTextColor(LCD_COLOR_PAPER_TRANSP);		
			BSP_LCD_FillRect(270, 18, 210, 152);		
			animation_en = 1;	
			}	
		BSP_LCD_SetTransparency(1, 255);
		animation_time = HAL_GetTick();	
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		BSP_LCD_DrawRect(0, 18, 14, 152);	
		for(E=0;E<4;E++)							/////Draw gray lines
			{
			BSP_LCD_FillRect(14, 18+(38*E), 466, 19);		
			}
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		for(E=0;E<4;E++)
			{
			BSP_LCD_FillRect(14, 37+(38*E), 466, 19);	
			}		
		BSP_LCD_SelectLayer(CURRENT_LAY);
		return;
		}
	else if(LAY==BR_NAVI_END)		//X => BROWSER NAVIGATION WITH ANIMATION END
		{
		CURRENT_LAY = ActiveLayer;		
		if(BROWSE_LEVEL==0) //in playlist 	
			{	
			BSP_LCD_SelectLayer(0);		
			if(BROWSER_INFO_enable)
				{
				intDrawLayer0_INFO_ANIMATION(B0CurrentCursorPosition);	
				HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_SET);					
				}
			else
				{
				intDrawLayer0_NOINFO_ANIMATION(B0CurrentCursorPosition);		
				}	
			BSP_LCD_SelectLayer(1);
			BSP_LCD_Clear(0x00000000);		
			BSP_LCD_SetTransparency(1, 255);
			BSP_LCD_SetFont(&Font15P);			
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);				
			sprintf((char*)Buf, "%s", "|");		
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);				
			sprintf((char*)Buf, "%s", TRACKLIST_NAME[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]);						
			BSP_LCD_DisplayStringAt(23, 2, Buf, TRANSPARENT_MODE);
			sprintf((char*)Buf, "%s", "Total Track");						
			BSP_LCD_DisplayStringAt(350, 2,Buf, TRANSPARENT_MODE);
			sprintf((char *)Buf , "%1lu", TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition] - TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]);				
			BSP_LCD_DisplayStringAt(448, 2,Buf, TRANSPARENT_MODE);	
			if(BROWSER_INFO_enable)
				{
				int_BIx_DRAW_ALL_LINES(0);	
				}
			else
				{
				int_B_DRAW_ALL_LINES();		
				}
			}
		else if(BROWSE_LEVEL==1) //playlists 	
			{
			BSP_LCD_SelectLayer(0);		
			intDrawLayer0_ANIMATION(B1CurrentCursorPosition);	
			BSP_LCD_SelectLayer(1);	
			BSP_LCD_Clear(0x00000000);	
			BSP_LCD_SetTransparency(1, 255);
			BSP_LCD_SetFont(&Font15P);			
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			sprintf((char*)Buf, "%s", "[PLAYLIST]");						
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
			int_BIx_DRAW_ALL_LINES(BROWSE_LEVEL);	
			HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);	
			}
		else if(BROWSE_LEVEL==2) //SD Card 	
			{
			BSP_LCD_SelectLayer(0);			
			intDrawLayer0_ANIMATION(B2CurrentCursorPosition);		
			BSP_LCD_SelectLayer(1);	
			BSP_LCD_Clear(0x00000000);		
			BSP_LCD_SetTransparency(1, 255);
			BSP_LCD_SetFont(&Font15P);	
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);	
			sprintf((char*)Buf, "%s", "/");		
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);				
			sprintf((char*)Buf, "%s", "SD card");						
			BSP_LCD_DisplayStringAt(23, 2, Buf, TRANSPARENT_MODE);								
			int_BIx_DRAW_ALL_LINES(BROWSE_LEVEL);		
			HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);		
			}	
		else if(BROWSE_LEVEL==3) //SD Card info 	
			{
			BSP_LCD_SelectLayer(0);			
			intDrawLayer0_ANIMATION(0);		
			BSP_LCD_SelectLayer(1);	
			BSP_LCD_Clear(0x00000000);		
			BSP_LCD_SetTransparency(1, 255);
			BSP_LCD_SetFont(&Font15P);			
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			sprintf((char*)Buf, "%s", "SD card slot");						
			BSP_LCD_DisplayStringAt(5,2,Buf, TRANSPARENT_MODE);
			int_BIx_DRAW_ALL_LINES(BROWSE_LEVEL);		
			HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);		
			}
		if(BROWSER_INFO_enable)
			{	
			dSHOW = BROWSER_INFO; 
			}
		else
			{
			dSHOW = BROWSER;	
			}
		BSP_LCD_SelectLayer(CURRENT_LAY);	
		return;	
		}
	}

//Redraw scroll on new position	
void ReDrawScroll(uint16_t total_elements, uint16_t current_element_pos)
	{
	ForceDrawVLine(5, ScrollPosition+24, ScrollLong-2, LCD_COLOR_BLACK);							//
	ForceDrawVLine(6, ScrollPosition+23, ScrollLong, LCD_COLOR_BLACK);								//	SCROLL
	ForceDrawVLine(7, ScrollPosition+23, ScrollLong, LCD_COLOR_BLACK);								//
	ForceDrawVLine(8, ScrollPosition+24, ScrollLong-2, LCD_COLOR_BLACK);							//
	
		///Calculate scroll	
	if(total_elements<9)
		{
		ScrollLong = 142;
		ScrollPosition = 0;	
		}
	else
		{
		ScrollLong = 1136/total_elements;
		if(ScrollLong>142)
			{
			ScrollLong = 142;	
			}
		else if(ScrollLong<5)
			{
			ScrollLong = 5;	
			}	
		ScrollPosition = ((current_element_pos-1)*(142-ScrollLong))/(total_elements-8);	
		}
	ForceDrawVLine(5, ScrollPosition+24, ScrollLong-2, LCD_COLOR_WHITE);							//
	ForceDrawVLine(6, ScrollPosition+23, ScrollLong, 	LCD_COLOR_WHITE);								//	SCROLL
	ForceDrawVLine(7, ScrollPosition+23, ScrollLong, 	LCD_COLOR_WHITE);								//
	ForceDrawVLine(8, ScrollPosition+24, ScrollLong-2, LCD_COLOR_WHITE);							//		
	}

//Navigate function for browser, taglis and utility
//
//	BROWSER0_DOWN		0 position track-- (cursor up)
//	BROWSER0_UP			1 position track++ (cursor down)
//	TAGLIST_DOWN		2	position track-- (cursor up)
//	TAGLIST_UP			3	position track++ (cursor down)
//	
void NAVIGATOR(uint8_t UPDOWN)
	{	
	if(dSHOW==BROWSER || dSHOW==BROWSER_INFO)		///////////////////////Browser mode selected
		{
		if(UPDOWN==BROWSER0_UP)
			{
			if(B0CurrentCursorPosition==7)												//All lines update++
				{
				if(BCurrentTrackPosition == TOTAL_TRACKS_IN_CURRENT_PLAYLIST-7)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);		
					BSP_LCD_SetFont(&Font15P);
					BCurrentTrackPosition++;
					if(dSHOW==BROWSER)
						{
						int_B_DRAW_ALL_LINES();
						}
					else
						{
						int_BIx_DRAW_ALL_LINES(0);	
						}
					return;
					}
				}
			else																								//One line update++
				{
				if(TOTAL_TRACKS_IN_CURRENT_PLAYLIST>8 || B0CurrentCursorPosition<(TOTAL_TRACKS_IN_CURRENT_PLAYLIST-1))
					{
					if(dSHOW==BROWSER)
						{	
						int_B_DRAW_ONE_LINE(UPDOWN);
						}
					else
						{
						int_BI_DRAW_ONE_LINE(UPDOWN);	
						}
					}
				return;
				}
			}
		else if(UPDOWN==BROWSER0_DOWN)
			{
			if(B0CurrentCursorPosition==0)											//All lines update--
				{
				if(BCurrentTrackPosition == 1)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);	
					BSP_LCD_SetFont(&Font15P);
					BCurrentTrackPosition--;
					if(dSHOW==BROWSER)
						{
						int_B_DRAW_ALL_LINES();
						}
					else
						{
						int_BIx_DRAW_ALL_LINES(0);	
						}	
					return;						
					}	
				}
			else														//One line update--
				{
				if(dSHOW==BROWSER)
					{	
					int_B_DRAW_ONE_LINE(UPDOWN);
					}
				else
					{
					int_BI_DRAW_ONE_LINE(UPDOWN);	
					}	
				return;			
				}	
			}	
		else if(UPDOWN==BROWSER1_UP)
			{				
			if(B1CurrentCursorPosition==7)												//All lines update++
				{
				if(BCurrentPlaylistPosition == TOTAL_TRACKLISTS-7)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);		
					BSP_LCD_SetFont(&Font15P);
					BCurrentPlaylistPosition++;
					int_BIx_DRAW_ALL_LINES(1);
					return;
					}
				}
			else																								//One line update++
				{
				if(TOTAL_TRACKLISTS>8 || B1CurrentCursorPosition<(TOTAL_TRACKLISTS-1))
					{
					int_B1_DRAW_ONE_LINE(UPDOWN);	 
					}
				return;
				}	
			}
		else if(UPDOWN==BROWSER1_DOWN)
			{
			if(B1CurrentCursorPosition==0)											//All lines update--
				{
				if(BCurrentPlaylistPosition == 1)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);	
					BSP_LCD_SetFont(&Font15P);
					BCurrentPlaylistPosition--;
					int_BIx_DRAW_ALL_LINES(1);	
					return;						
					}	
				}
			else														//One line update--
				{
				int_B1_DRAW_ONE_LINE(UPDOWN);	 
				return;			
				}	
			}	
		else if(UPDOWN==BROWSER2_UP)
			{
			if(B2CurrentCursorPosition==3)												//All lines update++
				{
				return;	
				}
			else																								//One line update++
				{
				int_B2_DRAW_ONE_LINE(UPDOWN);	 
				return;
				}			
			}
		else if(UPDOWN==BROWSER2_DOWN)
			{	
			if(B2CurrentCursorPosition==0)											//All lines update--
				{
				return;	
				}
			else														//One line update--
				{
				int_B2_DRAW_ONE_LINE(UPDOWN);	 
				return;			
				}		
			}
		return;
		}
	else if(dSHOW==TAG_LIST || dSHOW==TAG_LIST_INFO)	///////////////////TAG LIST mode selected
		{
		if(UPDOWN==TAGLIST_UP)
			{
			if(TCurrentCursorPosition==7)												//All lines update++
				{
				if(TCurrentTrackPosition == TOTAL_TRACKS_IN_TAG_LIST-7)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);		
					BSP_LCD_SetFont(&Font15P);
					TCurrentTrackPosition++;
					if(dSHOW==TAG_LIST)
						{
						int_T_DRAW_ALL_LINES();
						}
					else
						{
						int_TI_DRAW_ALL_LINES();	
						}
					return;
					}
				}
			else																								//One line update++
				{
				if(TOTAL_TRACKS_IN_TAG_LIST>8 || TCurrentCursorPosition<(TOTAL_TRACKS_IN_TAG_LIST-1))
					{
					if(dSHOW==TAG_LIST)
						{	
						int_T_DRAW_ONE_LINE(UPDOWN);
						}
					else
						{
						int_TI_DRAW_ONE_LINE(UPDOWN);	
						}
					}
				return;
				}
			}
		else if(UPDOWN==TAGLIST_DOWN)
			{
			if(TCurrentCursorPosition==0)											//All lines update--
				{
				if(TCurrentTrackPosition == 1)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);	
					BSP_LCD_SetFont(&Font15P);
					TCurrentTrackPosition--;
					if(dSHOW==TAG_LIST)
						{
						int_T_DRAW_ALL_LINES();
						}
					else
						{
						int_TI_DRAW_ALL_LINES();	
						}	
					return;						
					}	
				}
			else														//One line update--
				{
				if(dSHOW==TAG_LIST)
					{	
					int_T_DRAW_ONE_LINE(UPDOWN);
					}
				else
					{
					int_TI_DRAW_ONE_LINE(UPDOWN);	
					}	
				return;			
				}	
			}	
		return;	
		}
	else if(dSHOW==UTILITY)		///////////////////////UTILITY mode selected
		{
		if(UPDOWN==UTILITY_UP)
			{
			if(UCurrentCursorPosition==7)												//All lines update++
				{
				if(CurrentUPosition == TOTAL_U_POSITIONS-7)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);		
					BSP_LCD_SetFont(&Font15P);
					CurrentUPosition++;
					int_U_DRAW_ALL_LINES();
					return;
					}
				}
			else if(TOTAL_U_POSITIONS>8 || UCurrentCursorPosition<(TOTAL_U_POSITIONS-1))								//One line update++
				{
				int_U_DRAW_ONE_LINE(UPDOWN);
				return;
				}
			}
		else if(UPDOWN==UTILITY_DOWN)
			{
			if(UCurrentCursorPosition==0)											//All lines update--
				{
				if(CurrentUPosition == 1)
					{
					return;	
					}
				else
					{
					BSP_LCD_SelectLayer(1);	
					BSP_LCD_SetTransparency(1, 255);
					BSP_LCD_SetTextColor(0x00000000);
					BSP_LCD_FillRect(14, 18, 466, 152);	
					BSP_LCD_SetFont(&Font15P);
					CurrentUPosition--;
					int_U_DRAW_ALL_LINES();
					return;						
					}	
				}
			else														//One line update--
				{
				int_U_DRAW_ONE_LINE(UPDOWN);
				return;			
				}	
			}	
		return;
		}
	return;
	}
	
	
////////////////////////////////////////////////////
//draw triangle for browser with INFO
//
void intDrawTriangle(uint8_t CurrentCursorPosition)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_PAPER);
	BSP_LCD_DrawLine(270, 18, 270, 169)	;	
	BSP_LCD_SetTextColor(LCD_COLOR_SHADOW);
	BSP_LCD_DrawLine(271, 19, 271, 168);	
		
	for(uint16_t j = 0; j<304; j++)
		{
		BSP_LCD_DrawPixel(256+(j&0xF), ((36+(19*CurrentCursorPosition))-(j>>4)), (0xFF000000+strelka[j][0]+256*strelka[j][1]+65536*strelka[j][2]));
		}	
	BSP_LCD_DrawPixel(271, 18, LCD_COLOR_PAPER);	
	BSP_LCD_DrawPixel(271, 19, LCD_COLOR_SHADOW);	
	BSP_LCD_DrawPixel(271, 169, LCD_COLOR_PAPER);	
	BSP_LCD_DrawPixel(271, 168, LCD_COLOR_SHADOW);		
	return;
	}
	
/////////////////////////////////////////////////	
//
//draw layer 0 for INFO BROWSER and TAGLIST
//
void intDrawLayer0_INFO(uint8_t CurrentCursorPosition)
	{
	uint8_t E;	
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
	BSP_LCD_DrawRect(0, 18, 14, 152);	
	for(E=0;E<4;E++)															/////Draw gray lines
		{
		if(CurrentCursorPosition%2==1 | (CurrentCursorPosition/2) != E)
			{		
			BSP_LCD_FillRect(14, 18+(38*E), 256, 19);
			}			
		}
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
	for(E=0;E<4;E++)
		{
		if(CurrentCursorPosition%2==0 | (CurrentCursorPosition/2) != E)
			{				
			BSP_LCD_FillRect(14, 37+(38*E), 256, 19);
			}			
		}
		
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*CurrentCursorPosition)), 242, 9);			////Draw selected cursor
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*CurrentCursorPosition)), 242, 5);
			
	BSP_LCD_SetTextColor(LCD_COLOR_PAPER);					//Draw paper rectangle
	BSP_LCD_FillRect(270, 18, 210, 152);	
	BSP_LCD_SetTextColor(LCD_COLOR_SHADOW);					//Shadow
	BSP_LCD_DrawRect(271, 19, 208, 150);
	intDrawTriangle(CurrentCursorPosition);					//Draw triangle	
	for(uint16_t j = 0; j<7921; j++)					//Label
		{
		BSP_LCD_DrawPixel(384+(j%89), 120-j/89, (0xFF000000+disc[j][0]+256*disc[j][1]+65536*disc[j][2]));
		}		
	uint8_t j, k;
	for(j=0;j<7;j++)							////Dots
		{
		for(k=0;k<54;k++)
			{
			BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
			}
		}
	for(k=0;k<192;k++)					////ARTIST
		{
		if((iARTIST[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(295-k%16, 40+k/16, LCD_COLOR_DARK_1);
			}	
		}	
	for(k=0;k<192;k++)					////TIME
		{
		if((iTIME[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(295-k%16, 59+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<192;k++)					////BPM
		{
		if((iBPM[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 78+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<224;k++)					////TONE
		{
		if((iTONE[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 97+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<208;k++)					////DISC
		{
		if((iDISC[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 135+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<224;k++)					////COMENTS
		{
		if((iCOMMENTS[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 153+k/16, LCD_COLOR_DARK_1);
			}	
		}	
	return;	
	}

	
	
/////////////////////////////////////////////////	
//
//draw layer 0 for INFO BROWSER ANIMATION
//
void intDrawLayer0_INFO_ANIMATION(uint8_t CurrentCursorPosition)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
	BSP_LCD_DrawRect(0, 18, 14, 152);	

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*CurrentCursorPosition)), 242, 9);			////Draw selected cursor
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*CurrentCursorPosition)), 242, 5);
			
	BSP_LCD_SetTextColor(LCD_COLOR_PAPER);					//Draw paper rectangle
	BSP_LCD_FillRect(270, 18, 210, 152);	
	BSP_LCD_SetTextColor(LCD_COLOR_SHADOW);					//Shadow
	BSP_LCD_DrawRect(271, 19, 208, 150);
	intDrawTriangle(CurrentCursorPosition);					//Draw triangle	
	for(uint16_t j = 0; j<7921; j++)					//Label
		{
		BSP_LCD_DrawPixel(384+(j%89), 120-j/89, (0xFF000000+disc[j][0]+256*disc[j][1]+65536*disc[j][2]));
		}		
	uint8_t j, k;
	for(j=0;j<7;j++)							////Dots
		{
		for(k=0;k<54;k++)
			{
			BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
			}
		}
	for(k=0;k<192;k++)					////ARTIST
		{
		if((iARTIST[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(295-k%16, 40+k/16, LCD_COLOR_DARK_1);
			}	
		}	
	for(k=0;k<192;k++)					////TIME
		{
		if((iTIME[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(295-k%16, 59+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<192;k++)					////BPM
		{
		if((iBPM[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 78+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<224;k++)					////TONE
		{
		if((iTONE[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 97+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<208;k++)					////DISC
		{
		if((iDISC[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 135+k/16, LCD_COLOR_DARK_1);
			}	
		}
	for(k=0;k<224;k++)					////COMENTS
		{
		if((iCOMMENTS[k/16]>>(k%16))%2)
			{
			BSP_LCD_DrawPixel(294-k%16, 153+k/16, LCD_COLOR_DARK_1);
			}	
		}	
	return;	
	}	
	
	
	
	
/////////////////////////////////////////////////	
//
//draw layer 0 for animation 1-3 level
//
void intDrawLayer0_ANIMATION(uint8_t CurrentCursorPosition)
	{		
	uint8_t j, k;	
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*CurrentCursorPosition)), 242, 9);			////Draw selected cursor
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_PAPER);					//Draw paper rectangle
	BSP_LCD_FillRect(270, 18, 210, 152);	
	BSP_LCD_SetTextColor(LCD_COLOR_SHADOW);					//Shadow
	BSP_LCD_DrawRect(271, 19, 208, 150);
	intDrawTriangle(CurrentCursorPosition);					//Draw triangle	
		
	for(j=0;j<7;j++)							////Dots
		{
		for(k=0;k<99;k++)
			{
			BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
			}
		}	
	return;	
	}	
	
	
	
	
/////////////////////////////////////////////////	
//
//draw layer 0 for BROWSER 1-3 level
//
void intDrawLayer0_BROWSER_1_3(uint8_t CurrentCursorPosition)
	{		
	uint8_t E;	
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
	BSP_LCD_DrawRect(0, 18, 14, 152);	
	for(E=0;E<4;E++)															/////Draw gray lines
		{
		if(CurrentCursorPosition%2==1 | (CurrentCursorPosition/2) != E)
			{		
			BSP_LCD_FillRect(14, 18+(38*E), 256, 19);
			}			
		}
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
	for(E=0;E<4;E++)
		{
		if(CurrentCursorPosition%2==0 | (CurrentCursorPosition/2) != E)
			{				
			BSP_LCD_FillRect(14, 37+(38*E), 256, 19);
			}			
		}
			
	uint8_t j, k;	
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*CurrentCursorPosition)), 242, 9);			////Draw selected cursor
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_PAPER);					//Draw paper rectangle
	BSP_LCD_FillRect(270, 18, 210, 152);	
	BSP_LCD_SetTextColor(LCD_COLOR_SHADOW);					//Shadow
	BSP_LCD_DrawRect(271, 19, 208, 150);
	intDrawTriangle(CurrentCursorPosition);					//Draw triangle	
		
	for(j=0;j<7;j++)							////Dots
		{
		for(k=0;k<99;k++)
			{
			BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
			}
		}	
	return;	
	}		
	
	
	
	
	
//////////////////////////////////////////////////	
//	
//	draw layer 0 for without INFO BROWSER and TAGLIS	
void intDrawLayer0_NOINFO(uint8_t CurrentCursorPosition)
	{
	uint8_t E;
		
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
	BSP_LCD_DrawRect(0, 18, 14, 152);	
	for(E=0;E<4;E++)							/////Draw gray lines
		{
		if(CurrentCursorPosition%2==1 | (CurrentCursorPosition/2) != E)
			{		
			BSP_LCD_FillRect(14, 18+(38*E), 466, 19);
			}			
		}
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
	for(E=0;E<4;E++)
		{
		if(CurrentCursorPosition%2==0 | (CurrentCursorPosition/2) != E)
			{				
			BSP_LCD_FillRect(14, 37+(38*E), 466, 19);
			}			
		}

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*CurrentCursorPosition)), 466, 9);			////Draw selected cursor	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*CurrentCursorPosition)), 466, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*CurrentCursorPosition)), 466, 5);
	return;	
	}
	
	
//////////////////////////////////////////////////	
//	
//	draw layer 0 for without INFO BROWSER ANIMATION
void intDrawLayer0_NOINFO_ANIMATION(uint8_t CurrentCursorPosition)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
	BSP_LCD_DrawRect(0, 18, 14, 152);	
	
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*CurrentCursorPosition)), 466, 9);			////Draw selected cursor	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*CurrentCursorPosition)), 466, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*CurrentCursorPosition)), 466, 5);
	return;	
	}	
	
	
/////////////////////////////////	
//
//internal function for Browser
void int_B_DRAW_ALL_LINES(void)
	{
	uint16_t E;
	for(E=0;E<8 && TOTAL_TRACKS_IN_CURRENT_PLAYLIST>(E+BCurrentTrackPosition-1);E++)
		{					
		if(E==B0CurrentCursorPosition && (playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1][54]%2)==1)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
			}
		else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1][54]%2)==1)		
			{
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
			}
		else if(E==B0CurrentCursorPosition)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			}
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			}
		sprintf((char *)Buf , "%03lu", E+BCurrentTrackPosition);					
		BSP_LCD_DisplayStringAt(20,20+(19*E),Buf, TRANSPARENT_MODE);
		BSP_LCD_DisplayStringAt(69,20+(19*E), playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1], TRANSPARENT_MODE);
			
		if(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1] == track_play_now)
			{
			sprintf((char*)Buf, "%s", ">");	
			}
		else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1][54]&0x2)==0)
			{
			sprintf((char*)Buf, "%s", "~");  
			}	
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			sprintf((char*)Buf, "%s", "<");	
			}			
		BSP_LCD_DisplayStringAt(50,20+(19*E),Buf, TRANSPARENT_MODE);	
		}	
	BSP_LCD_SelectLayer(0);
	ReDrawScroll(TOTAL_TRACKS_IN_CURRENT_PLAYLIST, BCurrentTrackPosition);	
	}
/////////////////////////////////	
//
//internal function for Browser
void int_B_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	if(B0CurrentCursorPosition%2==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		}
	BSP_LCD_FillRect(14, 18+(19*B0CurrentCursorPosition), 466, 19);
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
		
	if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}		
	sprintf((char *)Buf , "%03lu", BCurrentTrackPosition+B0CurrentCursorPosition);					
	BSP_LCD_DisplayStringAt(20,20+(19*B0CurrentCursorPosition),Buf, TRANSPARENT_MODE);
			
	BSP_LCD_DisplayStringAt(69,20+(19*B0CurrentCursorPosition), playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1], TRANSPARENT_MODE);	

	if(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1] == track_play_now)
		{
		sprintf((char*)Buf, "%s", ">");	
		}	
	else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]&0x2)==0)
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf((char*)Buf, "%s", "<");		
		}
	BSP_LCD_DisplayStringAt(50,20+(19*B0CurrentCursorPosition),Buf, TRANSPARENT_MODE);	
	if(UPDOWN==1)
		{
		B0CurrentCursorPosition++;
		}
	else
		{
		B0CurrentCursorPosition--;	
		}
		
	if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}		
	sprintf((char *)Buf , "%03lu", BCurrentTrackPosition+B0CurrentCursorPosition);					
	BSP_LCD_DisplayStringAt(20,20+(19*B0CurrentCursorPosition),Buf, TRANSPARENT_MODE);	
	BSP_LCD_DisplayStringAt(69,20+(19*B0CurrentCursorPosition), playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1], TRANSPARENT_MODE);	
		
	if(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1] == track_play_now)	
		{
		sprintf((char*)Buf, "%s", ">");	
		}			
	else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]&0x2)==0)
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf((char*)Buf, "%s", "<");		
		}
	BSP_LCD_DisplayStringAt(50,20+(19*B0CurrentCursorPosition),Buf, TRANSPARENT_MODE);	
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*B0CurrentCursorPosition)), 466, 9);			//////////////////////////////	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*B0CurrentCursorPosition)), 466, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*B0CurrentCursorPosition)), 466, 5);
	return;			
	}	
	

/////////////////////////////////	
//
//internal function for Browser + INFO
void int_BIx_DRAW_ALL_LINES(uint8_t lvl)
	{
	uint16_t E, j, k;
	if(lvl==0)					//tracks
		{
		for(E=0;E<8 && TOTAL_TRACKS_IN_CURRENT_PLAYLIST>(E+BCurrentTrackPosition-1);E++)
			{				
			if(E==B0CurrentCursorPosition && (playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1][54]%2)==1)
				{
				BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
				}
			else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1][54]%2)==1)	
				{
				BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
				}
			else if(E==B0CurrentCursorPosition)
				{
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
				}
			else
				{
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				}
			BSP_LCD_DisplayStringAt(39,20+(19*E), playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1], INFO_MODE);		
				
			if(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1] == track_play_now)
				{
				sprintf((char*)Buf, "%s", ">");	
				}	
			else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+E+BCurrentTrackPosition-1]-1][54]&0x2)==0)	
				{
				sprintf((char*)Buf, "%s", "~");
				}		
			else
				{
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				sprintf((char*)Buf, "%s", "<");		
				}				
			BSP_LCD_DisplayStringAt(20,20+(19*E),Buf, INFO_MODE);		
			}	
		BSP_LCD_SetTextColor(0x00000000);													
		//BSP_LCD_FillRect(280, 21, 26, 13);
		BSP_LCD_FillRect(296, 59, 50, 15);
		BSP_LCD_FillRect(296, 78, 77, 15);
		BSP_LCD_FillRect(296, 97, 54, 15);
		BSP_LCD_FillRect(280, 116, 68, 13);	
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
		//sprintf((char *)Buf , "%03lu", TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]);		
		//BSP_LCD_DisplayStringAt(280, 21,Buf, TRANSPARENT_MODE);
		sprintf((char *)Buf , "%2lu"".""%1lu"" bpm", original_tempo[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]/10, 
			original_tempo[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]%10);		

		BSP_LCD_DisplayStringAt(296, 78, Buf, TRANSPARENT_MODE);
		int_VALUE_to_KEY(key_id[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]);	
		BSP_LCD_DisplayStringAt(296, 97, Buf, TRANSPARENT_MODE);
			
		sprintf((char *)Buf , "%02lu"":""%02lu", duration[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]/60, 
			duration[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]%60);		
		BSP_LCD_DisplayStringAt(296, 59, Buf, TRANSPARENT_MODE);		
		int_DRAW_STARS_RATING(rating[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]);
			
		BSP_LCD_SelectLayer(0);
		ReDrawScroll(TOTAL_TRACKS_IN_CURRENT_PLAYLIST, BCurrentTrackPosition);		
		}
	else if(lvl==1)						//playlists
		{	
		for(E=0;E<8 && TOTAL_TRACKLISTS>(E+BCurrentPlaylistPosition-1);E++)
			{
			if(E==B1CurrentCursorPosition)
				{
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
				}
			else
				{
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				}	
			BSP_LCD_DisplayStringAt(39,20+(19*E), TRACKLIST_NAME[E+BCurrentPlaylistPosition-1], INFO_MODE);		
			sprintf((char*)Buf, "%s", "|");		
			BSP_LCD_DisplayStringAt(20,20+(19*E),Buf, INFO_MODE);		
			}	
		BSP_LCD_SetTextColor(0x00000000);					//Draw paper rectangle
		BSP_LCD_FillRect(272, 18, 208, 152);						

		for(j=0;j<7;j++)							////Dots
			{
			for(k=0;k<99;k++)
				{
				BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
				}
			}	
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);	
		for(E=0;(E<8 && (TRACKLIST_OFFSET[BCurrentPlaylistPosition+B1CurrentCursorPosition] - TRACKLIST_OFFSET[BCurrentPlaylistPosition+B1CurrentCursorPosition-1])>E);E++)
			{
			sprintf((char*)Buf, "%s", "~");		
			BSP_LCD_DisplayStringAt(280, 21+(19*E),Buf, TRANSPARENT_MODE);	
			sprintf((char*)Buf, "%s", playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[BCurrentPlaylistPosition+B1CurrentCursorPosition-1]+E]-1]);						
			BSP_LCD_DisplayStringAt(299, 21+(19*E),Buf, TRANSPARENT_MODE);	
			}	
		BSP_LCD_SelectLayer(0);	
		ReDrawScroll(TOTAL_TRACKLISTS, BCurrentPlaylistPosition);		
		}
	else if(lvl==2)
		{
		for(E=0;E<4;E++)
			{
			if(E==B2CurrentCursorPosition)
				{
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
				}
			else
				{
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				}	
			if(E==0)
				{
				sprintf((char*)Buf, "%s", "[FILENAME]");			
				}
			else if(E==1)
				{
				sprintf((char*)Buf, "%s", "[FOLDER]");			
				}	
			else if(E==2)
				{
				sprintf((char*)Buf, "%s", "[PLAYLIST]");			
				}	
			else if(E==3)
				{
				sprintf((char*)Buf, "%s", "[TRACK]");			
				}				
			BSP_LCD_DisplayStringAt(20, 20+(19*E), Buf, INFO_MODE);			
			}
		BSP_LCD_SetTextColor(0x00000000);					//Draw paper rectangle
		BSP_LCD_FillRect(272, 18, 208, 152);	

		for(j=0;j<7;j++)							////Dots
			{
			for(k=0;k<99;k++)
				{
				BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
				}
			}	
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);	
			
		for(E=0;E<8;E++)
			{
			if(B2CurrentCursorPosition==0 || B2CurrentCursorPosition==3)
				{
				if(E<TOTAL_TRACKS)
					{
					BSP_LCD_DisplayStringAt(299,21+(19*E), playlist[E], TRANSPARENT_MODE);	
					sprintf((char*)Buf, "%s", "~");		
					BSP_LCD_DisplayStringAt(280, 21+(19*E),Buf, TRANSPARENT_MODE);		
					}
				}
			else if(B2CurrentCursorPosition==1)				
				{
				if(E==0)
					{
					sprintf((char*)Buf, "%s", "EMPTY");		
					BSP_LCD_DisplayStringAt(345, 59 ,Buf, TRANSPARENT_MODE);
					sprintf((char*)Buf, "%s", "or not support");		
					BSP_LCD_DisplayStringAt(319, 78 ,Buf, TRANSPARENT_MODE);	
					}
				}
			else if(B2CurrentCursorPosition==2)				
				{
				if(E<TOTAL_TRACKLISTS)
					{
					BSP_LCD_DisplayStringAt(299,21+(19*E), TRACKLIST_NAME[E], TRANSPARENT_MODE);
					sprintf((char*)Buf, "%s", "|");		
					BSP_LCD_DisplayStringAt(280, 21+(19*E),Buf, TRANSPARENT_MODE);		
					}
				}			
			}	
		BSP_LCD_SelectLayer(0);	
		ReDrawScroll(4, 0);	
		}	
	else if(lvl==3)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);				
		sprintf((char*)Buf, "%s", "/");		
		BSP_LCD_DisplayStringAt(20,20, Buf, TRANSPARENT_MODE);					
		sprintf((char*)Buf, "%s", "SD card");						
		BSP_LCD_DisplayStringAt(39, 20, Buf, TRANSPARENT_MODE);	
		BSP_LCD_SetTextColor(0x00000000);					//Draw paper rectangle
		BSP_LCD_FillRect(272, 18, 208, 152);	
		for(j=0;j<7;j++)							////Dots
			{
			for(k=0;k<99;k++)
				{
				BSP_LCD_DrawPixel(276+2*k, 36+19*j, LCD_COLOR_DARK_1);
				}
			}	
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);						
		BSP_LCD_DisplayStringAt(280, 21, SDCARD_NAME, TRANSPARENT_MODE);				//Flash name
		sprintf((char*)Buf, "%1lu"" songs", TOTAL_TRACKS);						
		BSP_LCD_DisplayStringAt(280, 40, Buf, TRANSPARENT_MODE);		
		sprintf((char*)Buf, "%1lu"" playlists", TOTAL_TRACKLISTS);						
		BSP_LCD_DisplayStringAt(280, 59, Buf, TRANSPARENT_MODE);	
		//sprintf((char*)Buf, "%s", "DATE");		
		BSP_LCD_DisplayStringAt(280, 78, SD_DATE, TRANSPARENT_MODE);			//DATE
		if(used_mem>999)
			{
			if(used_mem>9999)
				{
				sprintf((char*)Buf, "%1lu"".""%01lu"" GB used", used_mem/1000, (used_mem%1000)/100);		
				}
			else
				{
				sprintf((char*)Buf, "%1lu"".""%02lu"" GB used", used_mem/1000, (used_mem%1000)/10);		
				}
			}
		else
			{
			sprintf((char*)Buf, "%1lu"" MB used", used_mem);	
			}		
		BSP_LCD_DisplayStringAt(280, 97, Buf, TRANSPARENT_MODE);	
		if(free_mem>999)
			{	
			if(free_mem>9999)
				{
				sprintf((char*)Buf, "%1lu"".""%01lu"" GB free", free_mem/1000, (free_mem%1000)/100);		
				}
			else
				{
				sprintf((char*)Buf, "%1lu"".""%02lu"" GB free", free_mem/1000, (free_mem%1000)/10);		
				}
			}
		else
			{
			sprintf((char*)Buf, "%1lu"" MB free", free_mem);	
			}							
		BSP_LCD_DisplayStringAt(280, 116, Buf, TRANSPARENT_MODE);	
		BSP_LCD_SelectLayer(0);
		ReDrawScroll(1, 0);			
		}	
	return;	
	}
/////////////////////////////////	
//
//internal function for Browser + INFO
void int_BI_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	if(B0CurrentCursorPosition%2==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		}
	BSP_LCD_FillRect(14, 18+(19*B0CurrentCursorPosition), 256, 19);
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
	if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}		
	BSP_LCD_DisplayStringAt(39,20+(19*B0CurrentCursorPosition), playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1], INFO_MODE);	
	if(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1] == track_play_now)
		{
		sprintf((char*)Buf, "%s", ">");	
		}		
	else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]&0x2)==0)
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf((char*)Buf, "%s", "<");		
		}
	BSP_LCD_DisplayStringAt(20,20+(19*B0CurrentCursorPosition),Buf, INFO_MODE);		
	if(UPDOWN==1)
		{
		B0CurrentCursorPosition++;
		}
	else
		{
		B0CurrentCursorPosition--;	
		}
	if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]%2)==1)
		{			
		BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}	
	BSP_LCD_DisplayStringAt(39,20+(19*B0CurrentCursorPosition), playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1	], INFO_MODE);
	if(TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1] == track_play_now)		
		{
		sprintf((char*)Buf, "%s", ">");	
		}	
	else if((playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+BCurrentTrackPosition+B0CurrentCursorPosition-1]-1][54]&0x2)==0)	
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf((char*)Buf, "%s", "<");		
		}
	BSP_LCD_DisplayStringAt(20,20+(19*B0CurrentCursorPosition),Buf, INFO_MODE);	
	BSP_LCD_SetTextColor(0x00000000);				
	//BSP_LCD_FillRect(280, 21, 26, 13);
	BSP_LCD_FillRect(296, 59, 50, 15);
	BSP_LCD_FillRect(296, 78, 77, 15);
	BSP_LCD_FillRect(296, 97, 54, 15);
	BSP_LCD_FillRect(280, 116, 68, 13);	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
//	sprintf((char *)Buf , "%03lu", TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]);
//	BSP_LCD_DisplayStringAt(280, 21,Buf, TRANSPARENT_MODE);	
	sprintf((char *)Buf , "%2lu"".""%1lu"" bpm", original_tempo[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]/10, 
		original_tempo[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]%10);		
	BSP_LCD_DisplayStringAt(296, 78, Buf, TRANSPARENT_MODE);
	int_VALUE_to_KEY(key_id[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]);		
	BSP_LCD_DisplayStringAt(296, 97, Buf, TRANSPARENT_MODE);	
	sprintf((char *)Buf , "%02lu"":""%02lu", duration[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]/60, 
		duration[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]%60);		
	BSP_LCD_DisplayStringAt(296, 59, Buf, TRANSPARENT_MODE);	
	int_DRAW_STARS_RATING(rating[TRACKS_DATABASE[TRACKLIST_OFFSET[B1CurrentCursorPosition+BCurrentPlaylistPosition-1]+B0CurrentCursorPosition+BCurrentTrackPosition-1]-1]);
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*B0CurrentCursorPosition)), 242, 9);			//////////////////////////////	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*B0CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*B0CurrentCursorPosition)), 242, 5);
	intDrawTriangle(B0CurrentCursorPosition);			
	return;			
	}		
	
	
/////////////////////////////////	
//
//internal function for Browser level 1
void int_B1_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	uint8_t E;	
	if(B1CurrentCursorPosition%2==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		}
	BSP_LCD_FillRect(14, 18+(19*B1CurrentCursorPosition), 256, 19);
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(39,20+(19*B1CurrentCursorPosition), TRACKLIST_NAME[B1CurrentCursorPosition+BCurrentPlaylistPosition-1], INFO_MODE);		
	sprintf((char*)Buf, "%s", "|");		
	BSP_LCD_DisplayStringAt(20,20+(19*B1CurrentCursorPosition),Buf, INFO_MODE);	
	
	if(UPDOWN==7)
		{
		B1CurrentCursorPosition++;
		}
	else if(UPDOWN==6)
		{
		B1CurrentCursorPosition--;	
		}
		
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayStringAt(39,20+(19*B1CurrentCursorPosition), TRACKLIST_NAME[B1CurrentCursorPosition+BCurrentPlaylistPosition-1], INFO_MODE);		
	sprintf((char*)Buf, "%s", "|");		
	BSP_LCD_DisplayStringAt(20,20+(19*B1CurrentCursorPosition),Buf, INFO_MODE);	
		
	BSP_LCD_SetTextColor(0x00000000);					//Draw paper rectangle
	BSP_LCD_FillRect(272, 18, 208, 152);

	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);	
	for(E=0;(E<8 && (TRACKLIST_OFFSET[BCurrentPlaylistPosition+B1CurrentCursorPosition] - TRACKLIST_OFFSET[BCurrentPlaylistPosition+B1CurrentCursorPosition-1])>E);E++)
		{
		sprintf((char*)Buf, "%s", "~");		
		BSP_LCD_DisplayStringAt(280, 21+(19*E),Buf, TRANSPARENT_MODE);	
		sprintf((char*)Buf, "%s", playlist[TRACKS_DATABASE[TRACKLIST_OFFSET[BCurrentPlaylistPosition+B1CurrentCursorPosition-1]+E]-1]);						
		BSP_LCD_DisplayStringAt(299, 21+(19*E),Buf, TRANSPARENT_MODE);	
		}	
	
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*B1CurrentCursorPosition)), 242, 9);			//////////////////////////////	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*B1CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*B1CurrentCursorPosition)), 242, 5);
	intDrawTriangle(B1CurrentCursorPosition);			
	return;			
	}			
	
	
	/////////////////////////////////	
//
//internal function for Browser level 2
void int_B2_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	uint8_t E;	
	if(B2CurrentCursorPosition%2==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		}
	BSP_LCD_FillRect(14, 18+(19*B2CurrentCursorPosition), 256, 19);
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			
	if(B2CurrentCursorPosition==0)
		{
		sprintf((char*)Buf, "%s", "[FILENAME]");			
		}
	else if(B2CurrentCursorPosition==1)
		{
		sprintf((char*)Buf, "%s", "[FOLDER]");			
		}	
	else if(B2CurrentCursorPosition==2)
		{
		sprintf((char*)Buf, "%s", "[PLAYLIST]");			
		}	
	else if(B2CurrentCursorPosition==3)
		{
		sprintf((char*)Buf, "%s", "[TRACK]");			
		}				
	BSP_LCD_DisplayStringAt(20, 20+(19*B2CurrentCursorPosition), Buf, INFO_MODE);			


	if(UPDOWN==BROWSER2_UP)
		{
		B2CurrentCursorPosition++;
		}
	else if(UPDOWN==BROWSER2_DOWN)
		{
		B2CurrentCursorPosition--;	
		}

	BSP_LCD_SetTextColor(0x00000000);					//Draw paper rectangle
	BSP_LCD_FillRect(272, 18, 208, 152);	
	
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);	
		
	for(E=0;E<8;E++)
		{
		if(B2CurrentCursorPosition==0 || B2CurrentCursorPosition==3)
			{
			if(E<TOTAL_TRACKS)
				{
				BSP_LCD_DisplayStringAt(299,21+(19*E), playlist[E], TRANSPARENT_MODE);	
				sprintf((char*)Buf, "%s", "~");		
				BSP_LCD_DisplayStringAt(280, 21+(19*E),Buf, TRANSPARENT_MODE);		
				}	
			}
		else if(B2CurrentCursorPosition==1)				
			{
			if(E==0)
				{
				sprintf((char*)Buf, "%s", "EMPTY");		
				BSP_LCD_DisplayStringAt(345, 59 ,Buf, TRANSPARENT_MODE);	
				sprintf((char*)Buf, "%s", "or not support");		
				BSP_LCD_DisplayStringAt(319, 78 ,Buf, TRANSPARENT_MODE);	
				}
			}
		else if(B2CurrentCursorPosition==2)				
			{
			if(E<TOTAL_TRACKLISTS)
				{
				BSP_LCD_DisplayStringAt(299,21+(19*E), TRACKLIST_NAME[E], TRANSPARENT_MODE);
				sprintf((char*)Buf, "%s", "|");		
				BSP_LCD_DisplayStringAt(280, 21+(19*E),Buf, TRANSPARENT_MODE);		
				}
			}			
		}	
					
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		
	if(B2CurrentCursorPosition==0)
		{
		sprintf((char*)Buf, "%s", "[FILENAME]");			
		}
	else if(B2CurrentCursorPosition==1)
		{
		sprintf((char*)Buf, "%s", "[FOLDER]");			
		}	
	else if(B2CurrentCursorPosition==2)
		{
		sprintf((char*)Buf, "%s", "[PLAYLIST]");			
		}	
	else if(B2CurrentCursorPosition==3)
		{
		sprintf((char*)Buf, "%s", "[TRACK]");			
		}				
	BSP_LCD_DisplayStringAt(20, 20+(19*B2CurrentCursorPosition), Buf, INFO_MODE);				
			
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*B2CurrentCursorPosition)), 242, 9);			//////////////////////////////	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*B2CurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*B2CurrentCursorPosition)), 242, 5);
	intDrawTriangle(B2CurrentCursorPosition);			
	return;			
	}			
	
	
	
	
	
	
/////////////////////////////////	
//
//internal function for Browser + INFO 	
void int_DRAW_STARS_RATING(uint16_t rat)
	{
	uint16_t j, k;
	for(j=0;j<5;j++)							////STARS
		{		
		for(k=0;k<208;k++)
			{
			if((iSTAR[k/16]>>(k%16))%2)
				{
				BSP_LCD_DrawPixel((j*14)+294-k%16, 116+k/16, COLOR_MAP_RATING[rat&0x0F]);
				}	
			}
		if(((rat>>8)&0x07)>j)
			{
			for(k=0;k<208;k++)		////STARS FILLED
				{
				if((iSTAR_FILLED[k/16]>>(k%16))%2)
					{
					BSP_LCD_DrawPixel((j*14)+294-k%16, 116+k/16, COLOR_MAP_RATING[rat&0x0F]);
					}	
				}		
			}
		}
	return;	
	}
	
/////////////////////////////////	
//
//internal function for show KEY 	
void int_VALUE_to_KEY(uint8_t val)
	{
	if(val>25 || val==0)
		{
		sprintf((char*)Buf, "%s", " ");
		Buf[1] = 0;	
		return;		
		}
	sprintf((char*)Buf, "%s", KEYS[val-1]);	
	Buf[4] = 0;	
	return;	
	}
	
	
/////////////////////////////////	
//
//internal function for TAG LIST
void int_T_DRAW_ALL_LINES(void)
	{
	uint16_t E;
	for(E=0;E<8 && TOTAL_TRACKS_IN_TAG_LIST>(E+TCurrentTrackPosition-1);E++)
		{
		if(E==TCurrentCursorPosition && (TAG_LIST_BASE[E+TCurrentTrackPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[E+TCurrentTrackPosition-1]-1][54]%2)==1))
			{
			BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
			}
		else if(TAG_LIST_BASE[E+TCurrentTrackPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[E+TCurrentTrackPosition-1]-1][54]%2)==1)	
			{
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
			}
		else if(E==TCurrentCursorPosition)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			}
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			}
//		sprintf((char *)Buf , "%03lu", E+TCurrentTrackPosition);					
//		BSP_LCD_DisplayStringAt(20,20+(19*E),Buf, TRANSPARENT_MODE);
	
		if(TAG_LIST_BASE[E+TCurrentTrackPosition-1] == track_play_now)
			{
			sprintf((char*)Buf, "%s", ">");	
			}
		else
			{
			sprintf((char*)Buf, "%s", "~");
			}					
		BSP_LCD_DisplayStringAt(20,20+(19*E),Buf, TRANSPARENT_MODE);	
		BSP_LCD_DisplayStringAt(39,20+(19*E), playlist[TAG_LIST_BASE[E+TCurrentTrackPosition-1]-1], TRANSPARENT_MODE);		
		}	
	BSP_LCD_SelectLayer(0);
	ReDrawScroll(TOTAL_TRACKS_IN_TAG_LIST, TCurrentTrackPosition);	
	}
/////////////////////////////////	
//
//internal function for TAG LIST
void int_T_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	if(TCurrentCursorPosition%2==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		}
	BSP_LCD_FillRect(14, 18+(19*TCurrentCursorPosition), 466, 19);
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}		
//	sprintf((char *)Buf , "%03lu", TCurrentTrackPosition+TCurrentCursorPosition);					
//	BSP_LCD_DisplayStringAt(20,20+(19*TCurrentCursorPosition),Buf, TRANSPARENT_MODE);
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now)
		{
		sprintf((char*)Buf, "%s", ">");	
		}	
	else
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	BSP_LCD_DisplayStringAt(20,20+(19*TCurrentCursorPosition),Buf, TRANSPARENT_MODE);	
	BSP_LCD_DisplayStringAt(39,20+(19*TCurrentCursorPosition), playlist[TAG_LIST_BASE[TCurrentCursorPosition+TCurrentTrackPosition-1]-1], TRANSPARENT_MODE);		
	if(UPDOWN==TAGLIST_UP)
		{
		TCurrentCursorPosition++;
		}
	else
		{
		TCurrentCursorPosition--;	
		}
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}		
//	sprintf((char *)Buf , "%03lu", TCurrentTrackPosition+TCurrentCursorPosition);					
//	BSP_LCD_DisplayStringAt(20,20+(19*TCurrentCursorPosition),Buf, TRANSPARENT_MODE);	
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now)
		{
		sprintf((char*)Buf, "%s", ">");	
		}	
	else
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	BSP_LCD_DisplayStringAt(20,20+(19*TCurrentCursorPosition),Buf, TRANSPARENT_MODE);	
	BSP_LCD_DisplayStringAt(39,20+(19*TCurrentCursorPosition), playlist[TAG_LIST_BASE[TCurrentCursorPosition+TCurrentTrackPosition-1]-1], TRANSPARENT_MODE);
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*TCurrentCursorPosition)), 466, 9);			//////////////////////////////	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*TCurrentCursorPosition)), 466, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*TCurrentCursorPosition)), 466, 5);
	return;			
	}	
	

/////////////////////////////////	
//
//internal function for TAG LIST + INFO
void int_TI_DRAW_ALL_LINES(void)
	{
	uint16_t E;
	for(E=0;E<8 && TOTAL_TRACKS_IN_TAG_LIST>(E+TCurrentTrackPosition-1);E++)
		{
		if(E==TCurrentCursorPosition && (TAG_LIST_BASE[E+TCurrentTrackPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[E+TCurrentTrackPosition-1]-1][54]%2)==1))
			{
			BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
			}
		else if(TAG_LIST_BASE[E+TCurrentTrackPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[E+TCurrentTrackPosition-1]-1][54]%2)==1)	
			{
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
			}
		else if(E==TCurrentCursorPosition)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			}
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			}
			
		if(TAG_LIST_BASE[E+TCurrentTrackPosition-1] == track_play_now)
			{
			sprintf((char*)Buf, "%s", ">");	
			}
		else
			{
			sprintf((char*)Buf, "%s", "~");
			}					
		BSP_LCD_DisplayStringAt(20,20+(19*E),Buf, INFO_MODE);	
		BSP_LCD_DisplayStringAt(39,20+(19*E), playlist[TAG_LIST_BASE[E+TCurrentTrackPosition-1]-1], INFO_MODE);	
		}	
	BSP_LCD_SetTextColor(0x00000000);								////Draw track number and status (playing or played) in INFO mode
	//BSP_LCD_FillRect(280, 21, 26, 13);
	BSP_LCD_FillRect(296, 59, 50, 15);
	BSP_LCD_FillRect(296, 78, 77, 15);
	BSP_LCD_FillRect(296, 97, 54, 15);
	BSP_LCD_FillRect(280, 116, 68, 13);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		
	if(TOTAL_TRACKS_IN_TAG_LIST>0)
		{
		//sprintf((char *)Buf , "%03lu", TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]);		
		//BSP_LCD_DisplayStringAt(280, 21,Buf, TRANSPARENT_MODE);
		sprintf((char *)Buf , "%2lu"".""%1lu"" bpm", original_tempo[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]/10, original_tempo[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]%10);		
		BSP_LCD_DisplayStringAt(296, 78, Buf, TRANSPARENT_MODE);
		int_VALUE_to_KEY(key_id[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]);	
		BSP_LCD_DisplayStringAt(296, 97, Buf, TRANSPARENT_MODE);
		sprintf((char *)Buf , "%02lu"":""%02lu", duration[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]/60, duration[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]%60);		
		BSP_LCD_DisplayStringAt(296, 59, Buf, TRANSPARENT_MODE);		
		int_DRAW_STARS_RATING(rating[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]);		
		}
		
	BSP_LCD_SelectLayer(0);
	ReDrawScroll(TOTAL_TRACKS_IN_TAG_LIST, TCurrentTrackPosition);	
	}
/////////////////////////////////	
//
//internal function for TAG LIST + INFO
void int_TI_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	if(TCurrentCursorPosition%2==0)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
		}
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DARK_2);
		}
	BSP_LCD_FillRect(14, 18+(19*TCurrentCursorPosition), 256, 19);
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		}		
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now)
		{
		sprintf((char*)Buf, "%s", ">");	
		}	
	else
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	BSP_LCD_DisplayStringAt(20,20+(19*TCurrentCursorPosition),Buf, INFO_MODE);	
	BSP_LCD_DisplayStringAt(39,20+(19*TCurrentCursorPosition), playlist[TAG_LIST_BASE[TCurrentCursorPosition+TCurrentTrackPosition-1]-1], INFO_MODE);		
	if(UPDOWN==TAGLIST_UP)
		{
		TCurrentCursorPosition++;
		}
	else
		{
		TCurrentCursorPosition--;	
		}
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now || (playlist[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1][54]%2)==1)
		{
		BSP_LCD_SetTextColor(LCD_COLOR_DGREEN);	
		}	
	else
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		}		
	if(TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1] == track_play_now)
		{
		sprintf((char*)Buf, "%s", ">");	
		}	
	else
		{
		sprintf((char*)Buf, "%s", "~");
		}		
	BSP_LCD_DisplayStringAt(20,20+(19*TCurrentCursorPosition),Buf, INFO_MODE);	
	BSP_LCD_DisplayStringAt(39,20+(19*TCurrentCursorPosition), playlist[TAG_LIST_BASE[TCurrentCursorPosition+TCurrentTrackPosition-1]-1], INFO_MODE);
	BSP_LCD_SetTextColor(0x00000000);				
//	BSP_LCD_FillRect(280, 21, 26, 13);
	BSP_LCD_FillRect(296, 59, 50, 15);
	BSP_LCD_FillRect(296, 78, 77, 15);
	BSP_LCD_FillRect(296, 97, 54, 15);
	BSP_LCD_FillRect(296, 116, 68, 13);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//	sprintf((char *)Buf , "%03lu", TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]);		
//	BSP_LCD_DisplayStringAt(280, 21,Buf, TRANSPARENT_MODE);
	sprintf((char *)Buf , "%2lu"".""%1lu"" bpm", original_tempo[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]/10, original_tempo[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]%10);		
	BSP_LCD_DisplayStringAt(296, 78, Buf, TRANSPARENT_MODE);

	int_VALUE_to_KEY(key_id[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]);	
	BSP_LCD_DisplayStringAt(296, 97, Buf, TRANSPARENT_MODE);
	sprintf((char *)Buf , "%02lu"":""%02lu", duration[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]/60, duration[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]%60);		
	BSP_LCD_DisplayStringAt(296, 59, Buf, TRANSPARENT_MODE);		
	int_DRAW_STARS_RATING(rating[TAG_LIST_BASE[TCurrentTrackPosition+TCurrentCursorPosition-1]-1]);
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*TCurrentCursorPosition)), 242, 9);			//////////////////////////////	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*TCurrentCursorPosition)), 242, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*TCurrentCursorPosition)), 242, 5);
	intDrawTriangle(TCurrentCursorPosition);	
	return;			
	}		
	
	
	
/////////////////////////////////	
//
//internal function transparent bar for Browser, waveform
//
void int_DRAW_TRANSPARENT_BAR(void)
	{
	uint16_t j;
	uint32_t t, color;
	uint8_t r, g, b;	
	for(j=0;j<480;j++)
		{
		t = j;
		t = 160*t;
		if(t>0xFFFF)
			{
			t = 0xFFFF;	
			}
		t = 0xFFFF - t;	
		color = 0x01*t; 	
		r = color>>16;	
		color = 0xA6*t; 	
		g = color>>16;		
		color = 0xD4*t; 	
		b = color>>16;	
		color = 0xFF000000|r<<16|g<<8|b; 
		ForceDrawVLine(j, 0, 17, color);
		t = j;
		t = 150*t;
		if(t>0xFFFF)
			{
			t = 0xFFFF;	
			}
		t = 0xFFFF - t;	
		color = 0x53*t; 	
		r = color>>16;	
		color = 0xBD*t; 	
		g = color>>16;		
		color = 0xE9*t; 	
		b = color>>16;	
		color = 0xFF000000|r<<16|g<<8|b; 
		BSP_LCD_DrawPixel(j, 17, color);	
		}			
	return;	
	}	


/////////////////////////////////	
//
//internal function for UTILITY
void int_U_DRAW_ALL_LINES(void)
	{
	uint16_t E;
	for(E=0;E<8 && TOTAL_U_POSITIONS>(E+CurrentUPosition-1);E++)
		{
		if(E==UCurrentCursorPosition)
			{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);	
			}
		else
			{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			}
		BSP_LCD_DisplayStringAt(35, 21+(19*E), UTILITY_BASE[E+CurrentUPosition-1], TRANSPARENT_MODE);
		UTILITY_PARAMETER(E+CurrentUPosition-1);	
		BSP_LCD_DisplayStringAt(292, 21+(19*E),Buf, TRANSPARENT_MODE);			
		}	
	BSP_LCD_SelectLayer(0);
	ReDrawScroll(TOTAL_U_POSITIONS, CurrentUPosition);	
	}
/////////////////////////////////	
//
//internal function for UTILITY
void int_U_DRAW_ONE_LINE(uint8_t UPDOWN)							
	{
	BSP_LCD_SetTextColor(0xFF0F0F0);				
	BSP_LCD_FillRect(14, 18+(19*UCurrentCursorPosition), 256, 19);
	BSP_LCD_FillRect(271, 18+(19*UCurrentCursorPosition), 208, 19);	
	BSP_LCD_SetTextColor(LCD_COLOR_DARK_1);
	BSP_LCD_DrawLine(14,18+(19*UCurrentCursorPosition),479,18+(19*UCurrentCursorPosition));
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetFont(&Font15P);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(35, 21+(19*UCurrentCursorPosition), UTILITY_BASE[UCurrentCursorPosition+CurrentUPosition-1], TRANSPARENT_MODE);	
	UTILITY_PARAMETER(UCurrentCursorPosition+CurrentUPosition-1);	
	BSP_LCD_DisplayStringAt(292, 21+(19*UCurrentCursorPosition),Buf, TRANSPARENT_MODE);		

		
	if(UPDOWN==UTILITY_UP)
		{
		UCurrentCursorPosition++;
		}
	else
		{
		UCurrentCursorPosition--;	
		}
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayStringAt(35,21+(19*UCurrentCursorPosition), UTILITY_BASE[UCurrentCursorPosition+CurrentUPosition-1], TRANSPARENT_MODE);
	UTILITY_PARAMETER(UCurrentCursorPosition+CurrentUPosition-1);	
	BSP_LCD_DisplayStringAt(292, 21+(19*UCurrentCursorPosition),Buf, TRANSPARENT_MODE);				
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);		
	BSP_LCD_FillRect(14, (18+(19*UCurrentCursorPosition)), 256, 9);			////Draw selected cursor	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_2);
	BSP_LCD_FillRect(14, (27+(19*UCurrentCursorPosition)), 256, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_1);
	BSP_LCD_FillRect(14, (32+(19*UCurrentCursorPosition)), 256, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_3);		
	BSP_LCD_FillRect(271, (18+(19*UCurrentCursorPosition)), 208, 9);			////Draw selected cursor	
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_5);
	BSP_LCD_FillRect(271, (27+(19*UCurrentCursorPosition)), 208, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHT_4);
	BSP_LCD_FillRect(271, (32+(19*UCurrentCursorPosition)), 208, 5);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	FillTriangle(20, 20, 27, 20+(19*UCurrentCursorPosition), 34+(19*UCurrentCursorPosition), 27 +(19*UCurrentCursorPosition));		
	return;			
	}	
	

	
	
/////////////////////////////////	
//
//internal function for UTILITY for change parameter
void int_U_REDRAW_ONE_LINE(void)							
	{			
	BSP_LCD_SelectLayer(1);	
	BSP_LCD_SetTransparency(1, 255);	
	BSP_LCD_SetTextColor(0x00000000);	
	BSP_LCD_FillRect(271, 18+(19*UCurrentCursorPosition), 208, 19);		
	BSP_LCD_SetFont(&Font15P);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	UTILITY_PARAMETER(UCurrentCursorPosition+CurrentUPosition-1);	
	BSP_LCD_DisplayStringAt(292, 21+(19*UCurrentCursorPosition),Buf, TRANSPARENT_MODE);
	BSP_LCD_SelectLayer(0);		
	return;			
	}		
	
	
	
	
	

//////////////////////////////////////////
//			 "PLAY MODE           ",
//			 "LOAD LOCK           ",
//			 "AUTO CUE LEVEL      ",
//			 "TIME MODE DEFAULT   ",
//			 "TEMPO RANGE DEFAULT ",
//			 "SLIP FLASHING       ",
//			 "SWITCH REVERSE MODE ",
//			 "COLOR WAVEFORM      ",
//			 "LCD BRIGHTNESS      ",
//			 "JOG INDICATOR       ",
//			 "JOG BRIGHTNESS      ",
//			 "BPM COLOR						",
//			 "OUTPUT LEVEL        ",
//			 "DEVICE UID          ",
//			 "VERSION No.         "};	
//	
void UTILITY_PARAMETER(uint8_t num_parameter)
	{
	switch (num_parameter)
		{
		case 0:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "SINGLE");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "CONTINUE");
				}					
			break;	
			}
		case 1:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "UNLOCK");		
				}
			else
				{
				sprintf((char*)Buf, "%s", "LOCK");		
				}
			break;	
			}	
		case 2:
			{
			switch (UTILITY_SETTINGS[num_parameter])
				{
				case 0:	
					{
					sprintf((char*)Buf, "%s", "-36dB");	
					break;		
					}	
				case 1:	
					{
					sprintf((char*)Buf, "%s", "-42dB");	
					break;		
					}	
				case 2:	
					{
					sprintf((char*)Buf, "%s", "-48dB");	
					break;		
					}	
				case 3:	
					{
					sprintf((char*)Buf, "%s", "-54dB");	
					break;		
					}	
				case 4:	
					{
					sprintf((char*)Buf, "%s", "-60dB");	
					break;		
					}	
				case 5:	
					{
					sprintf((char*)Buf, "%s", "-66dB");	
					break;		
					}	
				case 6:	
					{
					sprintf((char*)Buf, "%s", "-72dB");	
					break;		
					}	
				case 7:	
					{
					sprintf((char*)Buf, "%s", "-78dB");	
					break;		
					}	
				case 8:	
					{
					sprintf((char*)Buf, "%s", "MEMORY");	
					break;		
					}	
				case 9:	
					{
					sprintf((char*)Buf, "%s", "FIRST BEAT");	
					break;		
					}	
			default: break;		
				}
			break;		
			}		
		case 3:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "ELAPSED");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "REMAIN");
				}					
			break;		
			}
		case 4:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "6%");	
				}	
			else if(UTILITY_SETTINGS[num_parameter]==1)
				{
				sprintf((char*)Buf, "%s", "10%");	
				}	
			else if(UTILITY_SETTINGS[num_parameter]==2)
				{
				sprintf((char*)Buf, "%s", "16%");	
				}	
			else
				{
				sprintf((char*)Buf, "%s", "WIDE");	
				}				
			break;			
			}
		case 5:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "OFF");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "ON");
				}					
			break;			
			}	
		case 6:
			{				
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "REVERSE");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "SLIP REVERSE");
				}					
			break;		
			}		
		case 7:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "BLUE");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "RGB");	
				}					
			break;			
			}
		case 8:
			{
			sprintf((char *)Buf , "%0lu", UTILITY_SETTINGS[num_parameter]+1);			
			break;	
			}		
		case 9:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "OFF");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "ON");
				}					
			break;	
			}	
		case 10:
			{
			sprintf((char *)Buf , "%0lu", UTILITY_SETTINGS[num_parameter]+1);			
			break;	
			}	
		case 11:
			{
			if(UTILITY_SETTINGS[num_parameter]==0)
				{
				sprintf((char*)Buf, "%s", "WHITE");	
				}		
			else
				{
				sprintf((char*)Buf, "%s", "ORANGE");
				}					
			break;	
			}	
		case 12:
			{
			sprintf((char *)Buf , "%01lu", (10*(UTILITY_SETTINGS[num_parameter]+1)));	
			break;	
			}		
		case 13:
			{
			sprintf((char *)Buf , "%08lX", DEVICE_UID);	
			break;	
			}		
		case 14:
			{
			sprintf((char*)Buf, "%s", "Ver. ");
			Buf[5] = FIRMWARE_VERSION[0];
			Buf[6] = FIRMWARE_VERSION[1];
			Buf[7] = FIRMWARE_VERSION[2];
			Buf[8] = FIRMWARE_VERSION[3];	
			Buf[9] = FIRMWARE_VERSION[4];					
			Buf[10] = 0;					
			break;	
			}		
		default:
			break;	
		}	
	return;	
	}

	
	
	
/////////////////////////////////	
//
//internal function for UTILITY for realtime reload parameters
//
//	
void int_reload_parameter_realtime(void)
	{
	if(UCurrentCursorPosition+CurrentUPosition==12)				//update BPM color
		{
		if(track_play_now==0)
			{
			ShowBPM(0xFFFF);		
			}
		else
			{
			ShowBPM(((originalBPM+5)*potenciometer_tempo)/100000);		
			}
		}		
	else if(UCurrentCursorPosition+CurrentUPosition==8)			//change waveform color
		{
		if(track_play_now!=0)
			{
			DrawStaticWFM(DRAW_NEW_STATIC_WAVEFORM);
			if(REMAIN_ENABLE)
				{
				DrawStaticWFM(REDRAW_IN_REMAIN_MODE);		
				}
			else
				{
				DrawStaticWFM(REDRAW_IN_NREMAIN_MODE);	
				}	
			DrawMinuteMarkers();	
			}
		}
	else if(UCurrentCursorPosition+CurrentUPosition==1)			//change single/track
		{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(47, 183, 34, 5);	
		BSP_LCD_SetFont(&FontBMP);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		if(UTILITY_SETTINGS[0]==0)			//single	
			{
			sprintf((char*)Buf, "%s", "CD");						//SINGLE	
			}
		else
			{
			sprintf((char*)Buf, "%s", "23");						//TRACK	
			}
		BSP_LCD_DisplayStringAt(47, 183, Buf, TRANSPARENT_MODE);	
		}		
	else if(UCurrentCursorPosition+CurrentUPosition==9)			//change lcd brightness
		{
		TIM14->CCR1 = 48+(52*UTILITY_SETTINGS[8]);
		}
	else if(UCurrentCursorPosition+CurrentUPosition==11)			//change jog brightness
		{
		TIM12->CCR1 = JOG_BRIGHTNESS[UTILITY_SETTINGS[10]];
		}	
	else if(UCurrentCursorPosition+CurrentUPosition==3)			//change auto cue level
		{
		if(UTILITY_SETTINGS[2]==8)
			{
			ShowACUE(2);	
			}
		else
			{
			ShowACUE(1);	
			}
		}		
	else if(UCurrentCursorPosition+CurrentUPosition==10)			//jog indicator
		{
		TIM12->CCR1 = JOG_BRIGHTNESS[UTILITY_SETTINGS[10]];
		}		
	}	
						

	

/////////////////////////////////	
//
//internal function for UTILITY for reload parameters
//
//	
void int_reload_parameter(void)
	{
	if(UCurrentCursorPosition+CurrentUPosition==13)			//change output level
		{			
		AUDIO_DrvTypeDef          *audio_drv;
		if(wm8994_drv.ReadID(AUDIO_I2C_ADDRESS) == WM8994_ID)
			{  
			audio_drv = &wm8994_drv; 
			audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_BOTH, (10*(UTILITY_SETTINGS[12]+1)), AUDIO_FREQUENCY_44K);
			}			
		}
	}		
	
	

///////////////////
//Encoder interrupt
//
//		
void EXTI15_10_IRQHandler(void)
	{
	if((HAL_GetTick()-encoder_tim)>10)
		{
		if(HAL_GPIO_ReadPin(GPIOA, ENC_A_Pin))
			{	
			need_up = 1;	
			}
		else
			{
			need_down = 1;	
			}
		}
	encoder_tim = HAL_GetTick();	
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);	
	}

///////////////////
//Timer 1000 Hz
//
//		
void TIM1_UP_TIM10_IRQHandler(void)
	{	
	if(change_speed==NEED_UP)
		{
		if(end_of_track)
			{
			change_speed = NO_CHANGE;
			pitch = 0;			
			}
		else
			{
			if(pitch<potenciometer_tempo-acceleration_UP)
				{
				pitch+=acceleration_UP;	
				}
			else
				{
				change_speed = NO_CHANGE;
				pitch = potenciometer_tempo;	
				}		
			}
		}
	else if(change_speed==NEED_DOWN)
		{
		if(end_of_track)
			{
			change_speed = NO_CHANGE;
			pitch = 0;			
			}
		else	
			{
			if(pitch>acceleration_DOWN)
				{
				pitch-=acceleration_DOWN;	
				}
			else
				{
				change_speed = NO_CHANGE;
				pitch = 0;	
				}		
			}
		}
				
	if(timer_time<124)
		{
		timer_time++;	
		}
	else
		{
		timer_time = 0;
				
		if(REALTIME_CUE_LED_BLINK<16)
			{
			if(REALTIME_CUE_LED_BLINK%4==0)
				{
				TIM_REALTIME_CUE_LED = 0;
				}
			else if(REALTIME_CUE_LED_BLINK%4==2)
				{
				TIM_REALTIME_CUE_LED = 1;
				}
			REALTIME_CUE_LED_BLINK++;	
			}
			
		LOOP_LEDS_BLINK++;					//loop leds	
			
		if(ENCODER_LED_BLINK<8)
			{	
			load_animation_enable = 1;	
			if(ENCODER_LED_BLINK%2==0)
				{
				HAL_GPIO_WritePin(GPIOB, LED_ENCODER_Pin, GPIO_PIN_RESET);	
				}
			else
				{
				HAL_GPIO_WritePin(GPIOB, LED_ENCODER_Pin, GPIO_PIN_SET);
				}
			ENCODER_LED_BLINK++;	
			}
		else
			{
			load_animation_enable = 0;	
			}
			
		if(LED_SD_timer<7)
			{
			LED_SD_timer++;		
			if(LED_SD_timer==4)
				{
				TIM_PLAY_LED = 1;
				TIM_CUE_LED = 0;		
				HAL_GPIO_WritePin(LED_SD_GPIO_Port, LED_SD_Pin, GPIO_PIN_SET);	
				}
			else if(LED_SD_timer==2 || LED_SD_timer==6)
				{
				TIM_CUE_LED = 1;					
				}	
			}
		else
			{
			if(track_play_now==0)
				{
				HAL_GPIO_WritePin(LED_SD_GPIO_Port, LED_SD_Pin, GPIO_PIN_SET);
				TIM_PLAY_LED = 0;	
				}
			else
				{
				TIM_PLAY_LED = 0;	
				TIM_CUE_LED = 0;	
				HAL_GPIO_WritePin(LED_SD_GPIO_Port, LED_SD_Pin, GPIO_PIN_RESET);	
				}
			LED_SD_timer = 0;	
			}
			
		if((LED_SD_timer==1 || LED_SD_timer==5) && track_play_now!=0)
			{
			if(((all_long-(play_adr/294))<4500) && end_of_track==0)
				{		
				if(DRAWN_IN_REMAIN==1)
					{
					need_DSW = 1;	
					}
				else
					{
					need_DSW = 2;	
					}
				}
			else	
				{	
				if((DRAWN_IN_REMAIN==0) && (REMAIN_ENABLE==1))
					{
					need_DSW = 2;	
					}
				else if((DRAWN_IN_REMAIN==1) && (REMAIN_ENABLE==0))
					{
					need_DSW = 1;			
					}
				if(TIM12->CCR1==0)	
					{
					TIM12->CCR1 = JOG_BRIGHTNESS[UTILITY_SETTINGS[10]];		
					}
				}		
			}	
		else if((LED_SD_timer!=1 && LED_SD_timer!=5) && (track_play_now!=0) && (all_long-(play_adr/294))<2250 && end_of_track==0)
			{	
			if(DRAWN_IN_REMAIN==1)
				{
				need_DSW = 1;	
				}
			else
				{
				need_DSW = 2;	
				}
			}	
			
		if(HAL_GPIO_ReadPin(KEY_MENU_GPIO_Port, KEY_MENU_Pin)==0  &&	dSHOW != UTILITY && countUTILITY<6)		///counter for long press MENU button
			{
			countUTILITY++;	
			}
		else
			{
			countUTILITY = 0;	
			}
		}
	
  HAL_TIM_IRQHandler(&htim1);
	}
////////////////////////////////
//TOUCH_SCREEN_HANDLER
//#define TS_NEEDLE_X_MIN		40
//#define TS_NEEDLE_X_MAX		439
//#define TS_NEEDLE_Y_MIN		235
//#define TS_NEEDLE_Y_MAX		271
void TOUCH_SCREEN_HANDLER(void)
	{
	uint16_t x=0, y=0;
	TS_GetState(&TS_State);
		
	if(TS_State.touchDetected)
		{
		x = TS_State.touchX[0];
		y = TS_State.touchY[0];

		if(tscnt[0]==1)						//Holding touch			
			{
			if(needle_enable)
				{
				if(y>=TS_NEEDLE_Y_MIN && y<=TS_NEEDLE_Y_MAX && x>=TS_NEEDLE_X_MIN && x<=TS_NEEDLE_X_MAX)
					{
					if(previous_needle_position != x)
						{
						previous_needle_position = x;
						SEEK_AUDIOFRAME(294*(((x-40)*all_long)/399));
						}	
					}
				else
					{
					ShowNEEDLE(0);	
					needle_enable = 0;
					forcibly_redraw = 1;	
					}
				}
			}
		else											//First touch
			{
			if(y>=TS_NEEDLE_Y_MIN && y<=TS_NEEDLE_Y_MAX && x>=(previous_position_bar+27) && x<=(previous_position_bar+53))	
				{
				ShowNEEDLE(1);	
				needle_enable = 1;
				forcibly_redraw = 1;	
				}
			tscnt[0] = 1;	
			}

		if (TS_State.touchDetected == 2)
			{
			x = TS_State.touchX[1];
			y = TS_State.touchY[1];
			if(tscnt[1]==1)						//Holding touch
				{

				}
			else											//First touch
				{
				tscnt[0] = 1;	
				}
			}
		}
	else
		{		
		tscnt[0]=0;
		tscnt[1]=0;
		if(needle_enable)
			{
			ShowNEEDLE(0);			
			needle_enable = 0;
			forcibly_redraw = 1;	
			}	
		}
	}
	
/////////////////////////////////////////////////////
//SPI transfer DMA handler 
//	
//
void DMA2_Stream5_IRQHandler(void)
	{	
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
	
	if(CheckRXCRC()==1)
		{
		uint32_t ptch;
		uint8_t acc_t;
			
		if((Rbuffer[14]&0x1) && PLAY_BUTTON_pressed==0)										///////////PLAY button
			{
			if(lock_control==0)	
				{
				if(CUE_BUTTON_pressed==0)
					{
					if(play_enable)
						{
						play_enable = 0;		
						change_speed = NEED_DOWN;
						}
					else
						{
						if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
							{	
							play_adr = slip_play_adr;	
							change_speed = NO_CHANGE;
							slip_play_enable = 1;	
							}
						else if(CUE_ADR!=(play_adr/294) && (Rbuffer[12]&0x20)==0)			//when playback starts from any adress and touch disable
							{
							change_speed = NEED_UP;
							}
						else																//when playback starts from CUE adress
							{
							change_speed = NO_CHANGE;	
							}
						play_enable = 1;	
						}
					}
				else
					{
					keep_to_play = 1;	
					}
				}
			PLAY_BUTTON_pressed = 1;	
			}
		else if((Rbuffer[14]&0x1)==0 && PLAY_BUTTON_pressed==1)
			{
			PLAY_BUTTON_pressed = 0;	
			}
		else if((Rbuffer[14]&0x2) && CUE_BUTTON_pressed==0)										///////////CUE button
			{
			if(lock_control==0)	
				{	
				if(play_enable && ((Rbuffer[12]&0x20)==0))								//return to CUE, when track playing				play && touch disable
					{
					pitch = 0;	
					play_enable = 0;		
					if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
						{	
						slip_play_enable = 0;		
						slip_play_adr = 294*CUE_ADR;	
						}	
					CUE_OPERATION = CUE_NEED_CALL;			
					}	
				else if((play_enable==0) && (CUE_ADR!=(play_adr/294)))			//Set new CUE, when track stopped		
					{
					LOOP_OUT = 0;	
					CUE_OPERATION = CUE_NEED_SET;		
					}
				else if((play_enable==0) && (CUE_ADR==(play_adr/294)))				//return to CUE adress, when track stopped
					{
					change_speed = NO_CHANGE;	
					//play_adr = 294*CUE_ADR;		
					if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
						{			
						slip_play_adr = play_adr;
						slip_play_enable = 1;	
						}	
					play_enable = 1;	
					}
				else if(play_enable && ((Rbuffer[12]&0x20)!=0) && (Tbuffer[19]&0x20))				//Set new CUE, when track played and press jog and JOG in Vinyl MODE
					{
					LOOP_OUT = 0;	
					CUE_OPERATION = CUE_NEED_SET;	
					play_enable = 0;		
					}
				}
			CUE_BUTTON_pressed = 1;	
			}
		else if((Rbuffer[14]&0x2)==0 && CUE_BUTTON_pressed==1)
			{
			if(lock_control==0)	
				{	
				if(keep_to_play==0)		//button play not pressed	
					{
					play_enable = 0;
					pitch = 0;		
					slip_play_enable = 0;	
					play_adr = 294*CUE_ADR;	
					if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
						{	
						slip_play_adr = play_adr;	
						}
		//			if((Tbuffer[19]&0x20)==0)   //CDJ mode
		//				{
		//				pitch = 0;	
		//				}
					}
				else
					{
					keep_to_play = 0;	
					}
				offset_adress = 0;																///	   temporary operation		
				}
			CUE_BUTTON_pressed = 0;	
			}
		else if((Rbuffer[14]&0x4) && REALTIME_CUE_BUTTON_pressed==0)										///////////REALTIME CUE button
			{
			if(lock_control==0)	
				{	
				if((play_enable==1) && (CUE_ADR!=(play_adr/294)) && loop_active==0)			//Set new CUE, when track play		
					{
					LOOP_OUT = 0;	
					CUE_OPERATION = CUE_NEED_SET;	
					}
				}
			REALTIME_CUE_BUTTON_pressed = 1;	
			}
		else if((Rbuffer[14]&0x4)==0 && REALTIME_CUE_BUTTON_pressed==1)
			{
			REALTIME_CUE_BUTTON_pressed = 0;	
			}		
			
			


		else if((Rbuffer[14]&0x08) && LOOP_OUT_BUTTON_pressed==0)										///////////LOOP OUT button
			{
			if(lock_control==0)	
				{	
				if(loop_active==0 && CUE_ADR<play_adr/294)
					{
					if(QUANTIZE && dSHOW==WAVEFORM)
						{
						if(((play_adr/294)>(BEATGRID[bars-1]+((BEATGRID[bars] - BEATGRID[bars-1])/2))) || bars==0)	
							{
							LOOP_OUT = BEATGRID[bars];								//next bar >> |
							}
						else
							{
							if(CUE_ADR==BEATGRID[bars-1])
								{
								LOOP_OUT = BEATGRID[bars];								//next bar >> |	
								}
							else	
								{
								LOOP_OUT = BEATGRID[bars-1];							//previous bar <<	|
								}
							CUE_OPERATION = CUE_NEED_CALL;		
							}	
						}
					else
						{
						LOOP_OUT = play_adr/294;
						CUE_OPERATION = CUE_NEED_CALL;	
						}	
					loop_active = 1;	
					}
				}
			LOOP_OUT_BUTTON_pressed = 1;	
			}
		else if((Rbuffer[14]&0x08)==0 && LOOP_OUT_BUTTON_pressed==1)
			{
			LOOP_OUT_BUTTON_pressed = 0;	
			}			
		else if((Rbuffer[14]&0x10) && RELOOP_BUTTON_pressed==0)										///////////RELOOP button
			{
			if(lock_control==0)	
				{		
				if(loop_active)
					{
					loop_active = 0;			
					}
				else if(loop_active==0 && CUE_ADR<LOOP_OUT)
					{
					loop_active = 1;	
					}
				if(dSHOW==WAVEFORM)							//Redraw cue on dynamic waveform
					{
					forcibly_redraw = 1;
					}		
				}	
			RELOOP_BUTTON_pressed = 1;	
			}
		else if((Rbuffer[14]&0x10)==0 && RELOOP_BUTTON_pressed==1)
			{
			RELOOP_BUTTON_pressed = 0;	
			}		
			
			
		if((Rbuffer[12]&0x2)==0 && REVERSE_SWITCH_pressed==0)					///////////reverse switch position
			{
			Tbuffer[17] |= 0x20;					//enable red led reverse
			if(UTILITY_SETTINGS[6]==1)						//SLIP+REVERSE MODE
				{
				if(Tbuffer[19]&0x8)			//SLIP MODE ENABLE
					{
					keep_slip = 1;	
					}
				else
					{
					Tbuffer[19] |= 0x8;	
					if(play_enable)
						{
						slip_play_enable = 1;	
						}
					slip_play_adr = play_adr; 	
					}					
				}
			REVERSE_SWITCH_pressed = 1;	
			}
		else if((Rbuffer[12]&0x2)!=0 && REVERSE_SWITCH_pressed==1)
			{				
			if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
				{	
				play_adr = slip_play_adr;	
				}		
			if(keep_slip)
				{
				keep_slip = 0;	
				}
			else if(UTILITY_SETTINGS[6]==1)					//SLIP MODE OFF
				{
				slip_play_enable = 0;	
				Tbuffer[19] &= 0xF7;	
				}
			Tbuffer[17] &= 0xDF;					//disable red led reverse
			REVERSE_SWITCH_pressed = 0;		
			}	
			
			
			///////////////////////////////////////////////////JOG MECHANICAL PROCESS///////////////////////////////////////////////
			
		if(inertial_rotation)
			{
			if(((Tbuffer[17]&0x20)==0 && (((Rbuffer[12]&0xC0)==0) || ((Rbuffer[12]&0x40) && pitch<potenciometer_tempo))) || 
				((Tbuffer[17]&0x20) && (((Rbuffer[12]&0xC0)==0x40) || ((Rbuffer[12]&0x40)==0 && pitch<potenciometer_tempo))))		//if rotation foward and stopped
				{
				inertial_rotation = 0;	
				}
			}	

			
		if(play_enable || (need_call_to_cue==3 && ((Rbuffer[12]&0x80)==0 || (Rbuffer[12]&0x20)!=0)))			//touch disable && rotation disable or play enable
			{
			need_call_to_cue = 0;	
			}	
		else if((Rbuffer[12]&0x20)==0 && need_call_to_cue==2)
			{
			pitch = 0;	
			play_adr = 294*CUE_ADR;	
			need_call_to_cue = 3;	
			}
		
		
		if(((Rbuffer[12]&0x20)!=0 || (play_enable==0 && (CUE_ADR!=(play_adr/294))) || inertial_rotation) && (Tbuffer[19]&0x20))				/////////////(touch enable	|| play_enable==0) && Vinyl mode enable
			{
			pitch_for_slip = potenciometer_tempo;	
			
			if(JOG_PRESSED==0)
				{
				if((Rbuffer[12]&0x20)!=0) 			//touch enable
					{
					change_speed = NEED_DOWN;	
					JOG_PRESSED = 2;	
					}
				else
					{
					JOG_PRESSED = 1;
					}				
				}	
				
			if(play_enable==0)	
				{
				if((Rbuffer[12]&0x20)!=0 && (CUE_ADR==(play_adr/294)) && need_call_to_cue==0)				//touch enable + play_enable==0 + CUE_ADR==(play_adr/294)
					{
					need_call_to_cue = 1;	
					}		
				//if(need_call_to_cue==1 && (Rbuffer[12]&0x20)==0 && play_enable==0 && (Rbuffer[12]&0x80)==0)			//touch disable		//problem on XDJ-RX2
				else if(need_call_to_cue==1 && (Rbuffer[12]&0x20)==0)			//touch disable	_/
					{
					need_call_to_cue = 2;	
					}					
				}
				
			if(Rbuffer[12]&0x80)					//rotation detect
				{
				if(need_call_to_cue<2)
					{					
					change_speed = NO_CHANGE;	
					
					if((Rbuffer[12]&0x40) && end_of_track)				//foward rotation + end_of_track
						{
						pitch = 0;
						change_speed = NO_CHANGE;	
						}
					else
						{
						ptch = (256*Rbuffer[10]+Rbuffer[11]);
						if(ptch<86)
							{
							ptch = 86;	
							}
						ptch = 5574324/ptch;
						pitch = ptch;						
						}
					pitch_for_slip = potenciometer_tempo; 	
					inertial_rotation = 1;						
					}
				}		
			else if(change_speed==NO_CHANGE) 
				{			
				pitch = 0;	
				}

			if(change_speed==NO_CHANGE)
				{	
				if(Rbuffer[12]&0x40)				//foward/reverse rotation
					{
					reverse = 0;
					}
				else
					{
					reverse = 1;	
					}		
				}
			else
				{
				if(Tbuffer[17]&0x20)					//reverse diode enable
					{
					reverse = 1;
					}
				else
					{
					reverse = 0;	
					}	
				}
			Tbuffer[23] |= 0x20;				//touch enable circle on display		
			}
		else if((Rbuffer[12]&0xA0)==0)				///////////////////////touch disable and rotation disable
			{			
			pitch_for_slip = potenciometer_tempo;
			if(JOG_PRESSED>0) //jog PRESSED -> UNPRESSED
				{
				if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
					{	
					change_speed = NO_CHANGE;	
					}
				else if(JOG_PRESSED==2 && play_enable)
					{
					change_speed = NEED_UP;	
					}					
				JOG_PRESSED = 0;	
				}
			if(play_enable)
				{	
				if(end_of_track && (Tbuffer[17]&0x20)==0)					//stop on end (to remove noise at the end of the track)
					{
					change_speed = NO_CHANGE;	
					pitch = 0;	
					}	
				else if(change_speed==NO_CHANGE)
					{
					pitch = potenciometer_tempo;	
					}
			
				if(Tbuffer[17]&0x20)					//reverse diode enable
					{
					reverse = 1;
					}
				else
					{
					reverse = 0;	
					}		
				}
			if(Tbuffer[23]&0x20)					//jog UNPRESSED
				{
				if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
					{	
					play_adr = slip_play_adr;	
					}	
				Tbuffer[23] &= 0xDF;				//disable touch circle on display
				}	
			}
		else if((Rbuffer[12]&0x80) && play_enable)						//rotation detected			(pitch bend)	
			{
			if(end_of_track==0)
				{
				ptch = (256*Rbuffer[10]+Rbuffer[11]);
				if(ptch>139)
					{
					ptch = ptch-139;	
					}
				else
					{
					ptch = 1;	
					}
				ptch = 150000/ptch;
					
				if(ptch>4225)
					{
					ptch = 4225;	
					}	
				if((Rbuffer[12]&0x40 && (Tbuffer[17]&0x20)==0) || ((Rbuffer[12]&0x40)==0 && Tbuffer[17]&0x20))		//foward rotation and reverse off OR reverse rotation and reverse on (pitch bend)			
					{
					ptch+= potenciometer_tempo;
					if(ptch>20000)
						{
						ptch = 20000;	
						}
					pitch = ptch;	
					}
				else if(((Rbuffer[12]&0x40)==0 && (Tbuffer[17]&0x20)==0) || (Rbuffer[12]&0x40 && Tbuffer[17]&0x20))	 //reverse rotation and reverse off OR foward rotation and reverse on(pitch bend)	
					{
					if(ptch<potenciometer_tempo)
						{
						pitch = potenciometer_tempo - ptch;
						}
					else
						{
						pitch = 0;	
						}
					}		
				}
			else
				{
				pitch = 0;		
				}			
				
			if(Tbuffer[17]&0x20)					//reverse diode enable
				{
				reverse = 1;
				}
			else
				{
				reverse = 0;	
				}	
			if(Tbuffer[23]&0x20)					//jog UNPRESSED
				{
				if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
					{	
					play_adr = slip_play_adr;	
					}	
				Tbuffer[23] &= 0xDF;				//disable touch circle on display	
				}	
			else
				{
				pitch_for_slip = pitch;		
				}
			}
			

		if(dma_cnt<5)									////////////////////////////////////////cicle divider////////////////////////////
			{
			dma_cnt++;		
			previous_adc_pitch+= (Rbuffer[7]+256*Rbuffer[6]);	
			previous_adc_DD+= (Rbuffer[5]+256*Rbuffer[4]);
			if(dma_cnt==1)
				{
				if(lock_control==0)	
					{	
					acc_t = Rbuffer[2]>>1;
					if(acc_t<4)
						{
						acceleration_DOWN = LOG_TABLE[0];	
						}
					else if(acc_t<12)
						{
						acceleration_DOWN = ((LOG_TABLE[1]*(acc_t-3)+LOG_TABLE[0]*(11-acc_t))>>3);	
						}
					else if(acc_t<28)
						{
						acceleration_DOWN = ((LOG_TABLE[2]*(acc_t-11)+LOG_TABLE[1]*(27-acc_t))>>4);	
						}
					else if(acc_t<44)
						{
						acceleration_DOWN = ((LOG_TABLE[3]*(acc_t-27)+LOG_TABLE[2]*(43-acc_t))>>4);	
						}
					else if(acc_t<76)
						{
						acceleration_DOWN = ((LOG_TABLE[4]*(acc_t-43)+LOG_TABLE[3]*(75-acc_t))>>5);	
						}
					else if(acc_t<92)
						{
						acceleration_DOWN = ((LOG_TABLE[5]*(acc_t-75)+LOG_TABLE[4]*(91-acc_t))>>4);	
						}
					else if(acc_t<108)
						{
						acceleration_DOWN = ((LOG_TABLE[6]*(acc_t-91)+LOG_TABLE[5]*(107-acc_t))>>4);	
						}	
					else if(acc_t<124)
						{
						acceleration_DOWN = ((LOG_TABLE[7]*(acc_t-107)+LOG_TABLE[6]*(123-acc_t))>>4);	
						}			
					else
						{
						acceleration_DOWN = 2;	
						}
					}
				}
			else if(dma_cnt==2)
				{
				if((Rbuffer[16]&0x4) && TRACK_NEXT_BUTTON_pressed==0) 							///////////TRACK NEXT Button
					{
					if(lock_control==0)	
						{	
						track_need_load = 1;
						}
					TRACK_NEXT_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[16]&0x4)==0 && TRACK_NEXT_BUTTON_pressed==1)	
					{
					TRACK_NEXT_BUTTON_pressed = 0;	
					}	
				else if((Rbuffer[16]&0x2) && TRACK_PREVIOUS_BUTTON_pressed==0) 			///////////TRACK PREVIOUS Button
					{
					if(lock_control==0)	
						{	
						track_need_load = 2;
						}
					TRACK_PREVIOUS_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[16]&0x2)==0 && TRACK_PREVIOUS_BUTTON_pressed==1)	
					{
					TRACK_PREVIOUS_BUTTON_pressed = 0;	
					}	
				else if((Rbuffer[18]&0x20) && TIME_MODE_BUTTON_pressed==0) 					///////////TIME MODE Button
					{	
					TIME_MODE_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[18]&0x20)==0 && TIME_MODE_BUTTON_pressed==1)	
					{
					if(REMAIN_ENABLE)
						{
						REMAIN_ENABLE = 0;	
						}
					else
						{
						REMAIN_ENABLE = 1;	
						}
					time_mode_need_update = 1;		
					TIME_MODE_BUTTON_pressed = 0;	
					}	
				else if((Rbuffer[18]&0x40) && QUANTIZE_BUTTON_pressed==0) 					///////////QUANTIZE Button
					{
					if(QUANTIZE)
						{
						QUANTIZE = 0;	
						}
					else
						{
						QUANTIZE = 1;	
						}	
					quantize_mode_need_update = 1;	
					QUANTIZE_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[18]&0x40)==0 && QUANTIZE_BUTTON_pressed==1)	
					{		
					QUANTIZE_BUTTON_pressed = 0;	
					}
				}
			else if(dma_cnt==3)
				{
				if(lock_control==0)	
					{	
					acc_t = Rbuffer[3]>>1;
					if(acc_t<4)
						{
						acceleration_UP = LOG_TABLE[0];	
						}
					else if(acc_t<12)
						{
						acceleration_UP = ((LOG_TABLE[1]*(acc_t-3)+LOG_TABLE[0]*(11-acc_t))>>3);	
						}
					else if(acc_t<28)
						{
						acceleration_UP = ((LOG_TABLE[2]*(acc_t-11)+LOG_TABLE[1]*(27-acc_t))>>4);	
						}
					else if(acc_t<44)
						{
						acceleration_UP = ((LOG_TABLE[3]*(acc_t-27)+LOG_TABLE[2]*(43-acc_t))>>4);	
						}
					else if(acc_t<76)
						{
						acceleration_UP = ((LOG_TABLE[4]*(acc_t-43)+LOG_TABLE[3]*(75-acc_t))>>5);	
						}
					else if(acc_t<92)
						{
						acceleration_UP = ((LOG_TABLE[5]*(acc_t-75)+LOG_TABLE[4]*(91-acc_t))>>4);	
						}
					else if(acc_t<108)
						{
						acceleration_UP = ((LOG_TABLE[6]*(acc_t-91)+LOG_TABLE[5]*(107-acc_t))>>4);	
						}	
					else if(acc_t<124)
						{
						acceleration_UP = ((LOG_TABLE[7]*(acc_t-107)+LOG_TABLE[6]*(123-acc_t))>>4);	
						}			
					else
						{
						acceleration_UP = 2;	
						}
					}
				}	
			else if(dma_cnt==4)
				{
				if((Rbuffer[16]&0x10) && SEARCH_FF_BUTTON_pressed==0) 							///////////SEARCH FF>> Button
					{
					if(lock_control==0)	
						{	
						if(play_enable & play_adr<(all_long+100000))	
							{
							//SEEK_AUDIOFRAME(play_adr+100000);	
							}
						else if(play_enable==0 & play_adr/294<(all_long+1))
							{	
							play_adr+=294;
							}
						}
					SEARCH_FF_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[16]&0x10)==0 && SEARCH_FF_BUTTON_pressed==1)	
					{
					SEARCH_FF_BUTTON_pressed = 0;	
					}	
				else if((Rbuffer[16]&0x8) && SEARCH_REW_BUTTON_pressed==0) 							///////////SEARCH <<REW Button
					{
					if(lock_control==0)	
						{		
						if(play_enable & play_adr>100000)	
							{
							//SEEK_AUDIOFRAME(play_adr-100000);
							}
						else if(play_enable==0 & play_adr>294)
							{	
							play_adr-=294;
							}
						}	
					SEARCH_REW_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[16]&0x8)==0 && SEARCH_REW_BUTTON_pressed==1)	
					{
					SEARCH_REW_BUTTON_pressed = 0;	
					}		

				if(track_play_now!=0)						//////////////////////////////LEDS/////////////////////////
					{
					Tbuffer[17] |= 0x2;				//CUE led on
						
					if(play_enable)
						{
						Tbuffer[17] |= 0x3;				//PLAY and CUE led on	
						}
					else											//Play led blink
						{	
						if(TIM_PLAY_LED)	
							{
							Tbuffer[17] |= 0x1;	
							}
						else
							{
							Tbuffer[17] &= 0xFE;	
							}
						if(TIM_CUE_LED)	
							{
							Tbuffer[17] |= 0x2;	
							}
						else if(CUE_ADR!=play_adr/294)  
							{
							Tbuffer[17] &= 0xFD;	
							}	
						}
						
					if(loop_active)
						{
						if(LOOP_LEDS_BLINK%4==0)	
							{
							Tbuffer[17] |= 0x0C;	
							}
						else if(LOOP_LEDS_BLINK%4==2)
							{
							Tbuffer[17] &= 0xF3;	
							}
						}	
					else
						{
						if(TIM_REALTIME_CUE_LED)	
							{
							Tbuffer[17] |= 0x0C;		
							}
						else
							{
							Tbuffer[17] &= 0xFB;	
							}				
						}				
					if(loop_active || CUE_ADR<LOOP_OUT)
						{
						Tbuffer[17] |= 0x10;							//RELOOP EXIT LED ON
						}
					else
						{
						Tbuffer[17] &= 0xEF;							//RELOOP EXIT LED OFF	
						}
					}
				}		
			else if(dma_cnt==5)
				{				
				if((Rbuffer[17]&0x4) && JOG_MODE_BUTTON_pressed==0) 							///////////Jog Mode button
					{
					if(Tbuffer[19]&0x20)			//VINYL => CDJ
						{
						Tbuffer[19] &= 0xDF;	
						Tbuffer[19] |= 0x40;
						Tbuffer[23] &= 0xE7;
						}	
					else												//CDJ => VINYL
						{
						Tbuffer[19] &= 0xBF;	
						Tbuffer[19] |= 0x20;
						Tbuffer[23] |= 0x18;	
						}
					JOG_MODE_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[17]&0x4)==0 && JOG_MODE_BUTTON_pressed==1)	
					{
					JOG_MODE_BUTTON_pressed = 0;	
					}
				else if((Rbuffer[17]&0x20) && TEMPO_RESET_BUTTON_pressed==0) 							///////////TEMPO RESET Button
					{
					if(Tbuffer[19]&0x10)				//ON_RESET => OFF_RESET
						{
						Tbuffer[19] &= 0xEF;
						}	
					else												//OFF_RESET => ON_RESET
						{
						Tbuffer[19] |= 0x10;
						}
					TEMPO_RESET_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[17]&0x20)==0 && TEMPO_RESET_BUTTON_pressed==1)	
					{
					TEMPO_RESET_BUTTON_pressed = 0;	
					}	
				else if((Rbuffer[17]&0x8) && TEMPO_RANGE_BUTTON_pressed==0) 							///////////TEMPO RANGE Button
					{
					if(tempo_range<3)
						{
						tempo_range++;
						}
					else
						{
						tempo_range = 0;	
						}
					tempo_range_need_update = 1;	
					TEMPO_RANGE_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[17]&0x8)==0 && TEMPO_RANGE_BUTTON_pressed==1)	
					{
					TEMPO_RANGE_BUTTON_pressed = 0;	
					}	
				else if((Rbuffer[18]&0x1) && CALL_NEXT_BUTTON_pressed==0) 							///////////CALL NEXT Button	>
					{
					if(lock_control==0)	
						{		
						CUE_OPERATION = MEMORY_NEED_NEXT_SET;
						}
					CALL_NEXT_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[18]&0x1)==0 && CALL_NEXT_BUTTON_pressed==1)	
					{
					CALL_NEXT_BUTTON_pressed = 0;	
					}
				else if((Rbuffer[18]&0x2) && CALL_PREVIOUS_BUTTON_pressed==0) 							///////////CALL PREVIOUS Button <
					{
					if(lock_control==0)	
						{		
						CUE_OPERATION = MEMORY_NEED_PREVIOUS_SET;
						}
					CALL_PREVIOUS_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[18]&0x2)==0 && CALL_PREVIOUS_BUTTON_pressed==1)	
					{
					CALL_PREVIOUS_BUTTON_pressed = 0;	
					}
					
				if((Rbuffer[17]&0x10) && SLIP_MODE_BUTTON_pressed==0) 							///////////SLIP MODE Button
					{
					if(Tbuffer[19]&0x8)					//ON_SLIP_MODE => OFF_SLIP_MODE
						{
						if(UTILITY_SETTINGS[6]==1 && keep_slip==0 && (Tbuffer[17]&0x20))
							{
							keep_slip = 1;				
							}
						else
							{	
							if(keep_slip)
								{
								keep_slip = 0;	
								}
							else
								{
								slip_play_enable = 0;	
								Tbuffer[19] &= 0xF7;
								}									
							}							
						}	
					else												//OFF_SLIP_MODE => ON_SLIP_MODE
						{	
						Tbuffer[19] |= 0x8;	
						if(play_enable)
							{
							slip_play_enable = 1;	
							}
						slip_play_adr = play_adr; 	
						}
					SLIP_MODE_BUTTON_pressed = 1;	
					}
				else if((Rbuffer[17]&0x10)==0 && SLIP_MODE_BUTTON_pressed==1)	
					{
					SLIP_MODE_BUTTON_pressed = 0;	
					}		
				}			
			}
		else
			{
			dma_cnt = 0;	
			if(Tbuffer[19]&0x10)				//TEMPRO RESET ON
				{
				potenciometer_tempo = 10000;	
				}
			else				//////////////////////////////////////////TEMPO CALCULATION
				{	
				previous_adc_pitch = previous_adc_pitch/5;
				previous_adc_DD = previous_adc_DD/5;
				if(previous_adc_pitch>(pitch_center+64) || (64+previous_adc_pitch)<pitch_center)		//
					{																																									//
					pitch_center = previous_adc_pitch;																								//
					}																																									//
				if(previous_adc_DD>DD+64 || 64+previous_adc_DD<DD)																	/////fluctuating filter with hysteresis
					{																																									//
					DD = previous_adc_DD;																															//
					}																																									//
				if(((pitch_center+128)>DD) && ((pitch_center-128)<DD))	
					{
					potenciometer_tempo = 10000;	
					}
				else if((pitch_center+128)<=DD)					/////pitch>0%
					{
					if(tempo_range==0)										//	6%
						{
						potenciometer_tempo = 2*((DD - pitch_center - 128)/108);
						if(potenciometer_tempo>600)
							{
							potenciometer_tempo = 600;	
							}	
						}	
					else if(tempo_range==1)										//	10%	
						{
						potenciometer_tempo = 5*((DD - pitch_center - 128)/162);
						if(potenciometer_tempo>1000)
							{
							potenciometer_tempo = 1000;	
							}
						}	
					else if(tempo_range==2)										//	16%	
						{
						potenciometer_tempo = 5*((DD - pitch_center - 128)/101);
						if(potenciometer_tempo>1600)
							{
							potenciometer_tempo = 1600;	
							}
						}	
					else																		//	WIDE	
						{
						potenciometer_tempo = 50*((DD - pitch_center - 128)/162);
						if(potenciometer_tempo>10000)
							{
							potenciometer_tempo = 10000;	
							}
						}	
					potenciometer_tempo = 10000 + potenciometer_tempo;	
					}
				else if((pitch_center-128)>=DD)				/////pitch<0%
					{
					if(tempo_range==0)										//	6%	
						{	
						potenciometer_tempo = 2*((pitch_center - 128 - DD)/108);
						if(potenciometer_tempo>600)
							{
							potenciometer_tempo = 600;	
							}
						}	
					else if(tempo_range==1)										//	10%	
						{	
						potenciometer_tempo = 5*((pitch_center - 128 - DD)/162);
						if(potenciometer_tempo>1000)
							{
							potenciometer_tempo = 1000;	
							}
						}
					else if(tempo_range==2)										//	16%	
						{	
						potenciometer_tempo = 5*((pitch_center - 128 - DD)/101);
						if(potenciometer_tempo>1600)
							{
							potenciometer_tempo = 1600;	
							}
						}	
					else																			//	WIDE
						{	
						potenciometer_tempo = 50*((pitch_center - 128 - DD)/162);
						if(potenciometer_tempo>10000)
							{
							potenciometer_tempo = 10000;	
							}
						}
					potenciometer_tempo = 10000 - potenciometer_tempo;	
					}				
				}
			if(previous_potenciometer_tempo != potenciometer_tempo)
				{
				previous_potenciometer_tempo = potenciometer_tempo;	
				tempo_need_update = 1;
				}
			previous_adc_DD = 0;
			previous_adc_pitch = 0;	
			}
				
					
		if(load_animation_enable)
			{
			Tbuffer[21] = 0;					//disable red cue marker
			Tbuffer[23] &= 0xDF;			//disable touch circle on display		
			Tbuffer[25] = 137;				//command load animation
			}
		else if(track_play_now==0)
			{
			Tbuffer[25] = 0;
			Tbuffer[21] = 0;	
			Tbuffer[17] &= 0xFC;			//PLAY & CUE leds off
			Tbuffer[23] &= 0xDF;				//disable touch circle on display		
			}
		else
			{
			if(TMPSLP)
				{
				Tbuffer[21] = RED_CIRCLE_CUE_ADR;		
				TMPSLP = 0;	
				}
			else
				{
				if(Tbuffer[19]&0x08)					//SLIP MODE ENABLE
					{	
					zi = ((slip_play_adr/588)%135)+1;	
					Tbuffer[21] = (1000*zi/1589)+1;	
					}
				TMPSLP = 1;	
				}	
			zi = ((play_adr/588)%135)+1;		
			Tbuffer[25] = zi;
			}
		}		
	CheckTXCRC();
	}	
	
	
	
/////////////////////////////////////////////	
//seek adress in samples (44100 per second)
//
void SEEK_AUDIOFRAME(uint32_t seek_adr)
	{
	if(seek_adr>(294*all_long))
		{
		return;	
		}
	seek_adr &= 0xFFFFE000;	

	if(FR_OK==f_lseek(&file, ((seek_adr<<2)+44)))
		{
		end_adr_valid_data = (seek_adr>>13);
		start_adr_valid_data = end_adr_valid_data; 	
		play_adr = seek_adr;	
		if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
			{	
			slip_play_adr = play_adr;	
			}			
		}
	return;	
	}

	
	
/////////////////////////////////////////////	
//SET CUE,create adress CUE_ADR
//create offset data
//copy audiodata from main audio buffer to cue_mem buffer
//
//	nf_adr - new frame adress in 1/150s
void SET_CUE(uint32_t nf_adr)
	{
	uint16_t c_adr;	
	uint32_t copy_cnt = 0;				//internal counter
	uint32_t AIS = 0;							//adress in samples 44k for CUE	
		
	CUE_ADR = nf_adr;	
	AIS = (294*CUE_ADR)&0xFFFFE000;							//rounding up to 8192
	AIS-=81920;
	mem_offset_adress = (AIS&0xFFFFF)>>13;
		
	for(copy_cnt=0;copy_cnt<327680;copy_cnt++)
		{
		PCM[(copy_cnt>>14)+128][(copy_cnt>>1)&0x1FFF][copy_cnt&0x01] = PCM[((copy_cnt>>14)+mem_offset_adress)&0x7F][(copy_cnt>>1)&0x1FFF][copy_cnt&0x01];	
		}
	DrawCueMarker(1+(400*CUE_ADR/all_long));	
	c_adr = ((nf_adr/2)%135)+1;	
	RED_CIRCLE_CUE_ADR = (1000*c_adr/1589)+1;
	if(dSHOW==WAVEFORM)							//Redraw cue on dynamic waveform
		{
		forcibly_redraw = 1;
		}	
	REALTIME_CUE_LED_BLINK = 0;					//start blink led
	return;	
	}

/////////////////////////////////////////////	
//SET CUE to MEMORY adress, create adress CUE_ADR
//create offset data
//copy audiodata from main audio buffer to cue_mem buffer
//
//	nf_adr - new frame adress in 1/150s	
void SET_MEMORY_CUE_1(uint32_t nf_adr)
	{
	uint16_t c_adr;	
	uint32_t AIS = 0;							//adress in samples 44k for CUE	
	pitch = 0;	
	play_enable = 0;
	if(Tbuffer[19]&0x8)					//OFF_SLIP_MODE
		{
		Tbuffer[19] &= 0xF7;	
		}	
		
	CUE_ADR = nf_adr;	
	AIS = (294*CUE_ADR)&0xFFFFE000;							//rounding up to 8192
	if(FR_OK==f_lseek(&file, ((AIS<<2)+44)))
		{
		end_adr_valid_data = (AIS>>13);
		start_adr_valid_data = end_adr_valid_data; 	
		play_adr = 294*CUE_ADR;		
		}

	AIS-=81920;
	mem_offset_adress = (AIS&0xFFFFF)>>13;	
	DrawCueMarker(1+(400*CUE_ADR/all_long));	
	c_adr = ((nf_adr/2)%135)+1;	
	RED_CIRCLE_CUE_ADR = (1000*c_adr/1589)+1;
	REALTIME_CUE_LED_BLINK = 0;					//start blink led		
	return;		
	}
	
/////////////////////////////////////////////	
//SET CUE to MEMORY adress second part operation
//Fill CUE buffer
//
//
void SET_MEMORY_CUE_2(void)
	{
	uint32_t copy_cnt = 0;				//internal counter
	for(copy_cnt=0;copy_cnt<327680;copy_cnt++)
		{
		PCM[(copy_cnt>>14)+128][(copy_cnt>>1)&0x1FFF][copy_cnt&0x01] = PCM[((copy_cnt>>14)+mem_offset_adress)&0x7F][(copy_cnt>>1)&0x1FFF][copy_cnt&0x01];	
		}	
	offset_adress = 128-mem_offset_adress;	
	return;	
	}
	
	
/////////////////////////////////////////////	
//CAL CUE,seek audio frame
//
//
//	
void CALL_CUE(void)
	{
	uint32_t seek_adr = 294*CUE_ADR;
	seek_adr &= 0xFFFFE000;	
	if(FR_OK==f_lseek(&file, ((seek_adr<<2)+44)))
		{
		end_adr_valid_data = (seek_adr>>13);
		start_adr_valid_data = end_adr_valid_data; 	
		play_adr = 294*CUE_ADR;		
		if(Tbuffer[19]&0x8)					//SLIP MODE ENABLE
			{	
			slip_play_adr = play_adr;	
			}			
		}
	offset_adress = 128-mem_offset_adress;	
	}
	
	
//////////////////////////////////////
//Function Checksum for TX package	
//
void CheckTXCRC()
	{
	uint8_t sdata = 141;
	uint8_t bt = 17;
	while(bt<26)
		{
		sdata+=Tbuffer[bt];	
		bt++;	
		}
	Tbuffer[26] = sdata;
	return;	
	}
	
	
//////////////////////////////////////
//Function Checksum for RX package	
//
uint8_t CheckRXCRC(void)
	{
	if(Rbuffer[0]==1 &&	Rbuffer[1]==16 && Rbuffer[25]==0 && Rbuffer[26]==0)
		{
		return 1;	
		}
	else
		{
		return 0;	
		}
	}
	
	
	
///////////////////////////////////////////////
//get free space and total space	
//		
void GET_SD_INFO(void)
	{
	FATFS *fs;
	DWORD fre_clust;	
	f_getfree("0:", &fre_clust, &fs);
	used_mem = (fs->n_fatent - 2) * fs->csize;
	used_mem = used_mem>>11;
	if(used_mem>99999)
		{
		used_mem = 99999;	
		}	
	free_mem = fre_clust * (fs->csize);
	free_mem = free_mem>>11;		
	if(free_mem>99999)
		{
		free_mem = 99999;	
		}	
	used_mem = used_mem - free_mem;
	return;	
	}	
	

	
//////////////////////////////////////////////////FatFs Functions//////////////////////
WCHAR ff_convert (WCHAR wch, UINT dir) 
{ 
          if (wch < 0x80) { 
                    /* ASCII Char */ 
                    return wch; 
          }  

          /* I don't support unicode it is too big! */ 
          return 0; 
}  

WCHAR ff_wtoupper (WCHAR wch) 
{ 
          if (wch < 0x80) {      
                    /* ASCII Char */ 
                    if (wch >= 'a' && wch <= 'z') { 
                              wch &= ~0x20; 
                     } 
                      return wch; 
          }  

          /* I don't support unicode it is too big! */ 
          return 0; 
} 
//////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
