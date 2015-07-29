#include "ch.h"
#include "hal.h"
#include "i2c.h"


#include "audio.h"
#include "gpiox.h"
#include "orchard.h"
#include "orchard-events.h"

#include "orchard-test.h"
#include "test-audit.h"


#define LENGTH_REG_PROGM 2
#define LENGTH_REG_PARAM 4

//IMPORTANT!
//check signal in vs. signal out for polarity.  
//INVERSION happens at PGA? if enabled and with DAC_CTRL_01 REGISTER on output to compensate
//currently, PGA muted so inversion not assumed and none applied at DAC

//check MP registers, set on uC side. 
//right now we have: 
//MP0 Serial Input 0 - pin 21 PTB2
//MP1 pb vol up - pin 18 PTB1
//MP2 bit clock - pin 22 PTB17
//MP3 LR clock - pin 26 PTC3
//MP4 dig mic in (GND)
//MP5 dig mic in (GND)
//MP6 pb vol down - pin 14 PTA1

//could be issue with decimator power control register if nothing on input

//can meter input level and output level at core
//using REG_DBREG0-1
//can switch PB_VOL_CONTROL to control gain at ADCs or DACs
//using PB_VOL_CONV

#define REG_CLK_CTRL 0x0000
#define REG_CLK_CTRL_SETUP 0x00 //write to PLL CNTRL registers before starting the core
								// clock by setting this register to DEFAULT
#define REG_CLK_CTRL_DEFAULT 0x07 //disable PLL, enable XTAL, set clk dividers
								   // enable I2C spike filter, enable core clk
//check on clock divisors

#define REG_PLL_CTRL0 0x0001 //PLL Denominator (Fractional Setting)
#define REG_PLL_CTRL1 0x0002 //PLL Denominator (Fractional Setting)
#define REG_PLL_CTRL2 0x0003 //PLL Numerator (Fractional Setting)
#define REG_PLL_CTRL3 0x0004 //PLL Numerator (Fractional Setting)
#define REG_PLL_CTRL4 0x0005 //PLL Integer/Fractional Set, Integer Settings
#define REG_PLL_CTRL5 0x0006 // PLL Lock (read-only)

#define REG_PLL_CTRL_DEFAULT 0x00 //set all to 0, PLL unused. Default RESET state

#define REG_CLKOUT_SEL 0x0007 //Set CLKout freq relative to core clock on MP6 if set
#define REG_CLKOUT_DEFAULT 0x07  //set to off

#define REG_REGULATOR 0x0008
#define REG_REGULATOR_DEFAULT 0x00 //set to 1.2V output on LDO pin and active

#define REG_CORE_CTRL 0x0009
#define REG_CORE_CTRL_ZEROSTATE_ON (1 << 7)
#define REG_CORE_CTRL_ZEROSTATE_OFF (0 << 7)
#define REG_CORE_CTRL_BANK_A ((0 << 6) | (0 << 5))
#define REG_CORE_CTRL_BANK_B ((0 << 6) | (1 << 5))
#define REG_CORE_CTRL_96k ((0 << 2) | (1 << 1))
#define REG_CORE_CTRL_192k ((1 << 2) | (0 << 1))
#define REG_CORE_CTRL_STARTCORE (1 << 0)
#define REG_CORE_CTRL_DEFAULT (REG_CORE_CTRL_ZEROSTATE_ON | REG_CORE_CTRL_BANK_A | REG_CORE_CTRL_192k | REG_CORE_CTRL_STARTCORE)

#define REG_CORE_ENABLE 0x000B
#define REG_CORE_ENABLE_ENABLELIMITER (1 << 1)
#define REG_CORE_ENABLE_DISABLELIMITER (0 << 1)
#define REG_CORE_ENABLE_DSPCLK (1 << 0) //enable DSP (use the chip for more than ADC/DAC)
										//must be enabled before writing coeffs
#define REG_CORE_ENABLE_DEFAULT (REG_CORE_ENABLE_ENABLELIMITER | REG_CORE_ENABLE_DSPCLK)

#define REG_DBREG0 0x000C //read-only DB value (can be set to RMS of an input)
#define REG_DBREG1 0x000D //read-only DB value (can be set to RMS of an input)
#define REG_DBREG2 0x000E //read-only DB value (can be set to RMS of an input)

#define REG_COREIN_01 0x000F //input select channel 0/1
#define REG_COREIN_01_AIN1T01 (0x1 << 4)
#define REG_COREIN_01_AIN0TO0 (0x0 << 0)
#define REG_COREIN_01_DEFAULT (REG_COREIN_01_AIN1T01 | REG_COREIN_01_AIN0TO0)

#define REG_COREIN_23 0x0010 //input select channel 0/1
#define REG_COREIN_23_AIN3TO3 (0x3 << 4)
#define REG_COREIN_23_AIN2TO2 (0x2 << 0)
#define REG_COREIN_23_DEFAULT (REG_COREIN_23_AIN3TO3 | REG_COREIN_23_AIN2TO2)

#define REG_DAC_SOURCE_01 0x0011
#define REG_DAC_SOURCE_01_1TODAC1 (0x1 << 4)
#define REG_DAC_SOURCE_01_0TODAC0 (0x0 << 0)
#define REG_DAC_SOURCE_01_DEFAULT (REG_DAC_SOURCE_01_1TODAC1 | REG_DAC_SOURCE_01_0TODAC0)

#define REG_PDM_SOURCE_01 0x0012 //leave as default for now
#define REG_PDM_SOURCE_01_3TOPDM1 (0x3 << 4)
#define REG_PDM_SOURCE_01_2TOPDM0 (0x2 << 0)
#define REG_PDM_SOURCE_01_DEFAULT (REG_PDM_SOURCE_01_3TOPDM1 | REG_PDM_SOURCE_01_2TOPDM0)

#define REG_SDATAOUT_01 0x0013 //leave as default for now
#define REG_SDATAOUT_23 0x0014 //leave as default for now
#define REG_SDATAOUT_45 0x0015 //leave as default for now
#define REG_SDATAOUT_67 0x0016 //leave as default for now

#define REG_ADC_SDATA_01 0x0017 //leave as default for now

#define REG_ASRC_OUT_01 0x0018 //leave as default for now
#define REG_ASRC_OUT_23 0x0019 //leave as default for now

#define REG_ASRC_MODE 0x001A
#define REG_ASRC_MODE_DEFAULT 0x00 //disable ASRC

#define REG_ADC_CTRL_01 0x001B //unmute and 192k
#define REG_ADC_CTRL_01_MUTE ((1 << 4) | (1 << 3))
#define REG_ADC_CTRL_01_UNMUTE ((0 << 4) | (0 << 3))
#define REG_ADC_CTRL_01_96k ((0 << 1) | (0 << 0))
#define REG_ADC_CTRL_01_192k ((0 << 1) | (1 << 0))
#define REG_ADC_CTRL_01_DEFAULT (REG_ADC_CTRL_01_UNMUTE | REG_ADC_CTRL_01_192k)

#define REG_ADC_CTRL_23 0x001C //leave muted (default)
#define REG_ADC_CTRL_23_MUTE ((1 << 4) | (1 << 3))
#define REG_ADC_CTRL_23_UNMUTE ((0 << 4) | (0 << 3))
#define REG_ADC_CTRL_23_96k ((0 << 1) | (0 << 0))
#define REG_ADC_CTRL_23_192k ((0 << 1) | (1 << 0))
#define REG_ADC_CTRL_23_DEFAULT (REG_ADC_CTRL_01_MUTE | REG_ADC_CTRL_01_192k)

#define REG_ADC_CTRL1_01 0x001D
#define REG_ADC_CTRL1_01_HPFOFF ((0 << 6) | (0 << 5))
#define REG_ADC_CTRL1_01_HPF1HZ ((0 << 6) | (1 << 5))
#define REG_ADC_CTRL1_01_HPF4HZ ((1 << 6) | (0 << 5))
#define REG_ADC_CTRL1_01_HPF8HZ ((1 << 6) | (1 << 5))
#define REG_ADC_CTRL1_01_SRCADC ((0 << 3) | (0 << 2))
#define REG_ADC_CTRL1_01_SRCDIGMIC ((1 << 3) | (1 << 2))
#define REG_ADC_CTRL1_01_ADCENABLED ((1 << 1) | (1 << 0))
#define REG_ADC_CTRL1_01_ADCDISABLED ((0 << 1) | (0 << 0))
#define REG_ADC_CTRL1_01_DEFAULT (REG_ADC_CTRL1_01_HPF8HZ | REG_ADC_CTRL1_01_SRCADC | REG_ADC_CTRL1_01_ADCENABLED)

#define REG_ADC_CTRL1_23 0x001E //leave as default, turned off
//same behavior as REG_ADC_CTRL1_01, but for channels 2/3

#define REG_ADC0_VOL 0x001F //0b00000000 is 0dBFS
#define REG_ADC1_VOL 0x0020	//0b11111111 is -95.625dBFS (.375dB increments)
#define REG_ADC2_VOL 0x0021
#define REG_ADC3_VOL 0x0022
//leaving all as default 0dbFS for now until clipping on input is heard

#define REG_PGAAIN0_CTRL 0x0023
#define REG_PGAAIN1_CTRL 0x0024
#define REG_PGAAIN2_CTRL 0x0025
#define REG_PGAAIN3_CTRL 0x0026

#define REG_PGA_CTRL_MIC (1 << 7)
#define REG_PGA_CTRL_LINE (0 << 7)
#define REG_PGA_CTRL_ENABLEPGA (0 << 6)
#define REG_PGA_CTRL_DISABLEPGA (1 << 6)
#define REG_PGA_CTRL_M12DB 0x00
#define REG_PGA_CTRL_M6DB 0x08   
#define REG_PGA_CTRL_0DB 0x10
#define REG_PGA_CTRL_6DB 0x18
#define REG_PGA_CTRL_12DB 0x20
#define REG_PGA_CTRL_20DB 0x2B
#define REG_PGA_CTRL_35DB 0x3F
//leave as default for muted PGAs 2/3, set to MIC for 0/1 just in case
//can enable and add/remove gain after clipping/headroom is determined
#define REG_PGA_CTRL_DEFAULT (REG_PGA_CTRL_MIC | REG_PGA_CTRL_DISABLEPGA)


#define REG_PGASLEW_CTRL 0x0027 //can leave as default for now (on for all channels at 21ms)
#define REG_PGASLEW_CTRL_RATE21MS ((0 << 5) | (0 << 4))
#define REG_PGASLEW_CTRL_RATE42MS ((0 << 5) | (1 << 4))
#define REG_PGASLEW_CTRL_RATE85MS ((1 << 5) | (0 << 4))
#define REG_PGASLEW_CTRL_01ENABLE ((0 << 1) | (0 << 0))
#define REG_PGASLEW_CTRL_01DISABLE ((1 << 1) | (1 << 0))
#define REG_PGASLEW_CTRL_23ENABLE ((0 << 3) | (0 << 2))
#define REG_PGASLEW_CTRL_23DISABLE ((1 << 3) | (1 << 2))
#define REG_PGASLEW_CTRL_DEFAULT (REG_PGASLEW_CTRL_RATE42MS | REG_PGASLEW_CTRL_01ENABLE | REG_PGASLEW_CTRL_23DISABLE)

#define REG_PGA_10DB_BOOST 0x0028 //leave as default 0dB, can add 10dB to each PGA
#define REG_PGA_10DB_BOOST_BOOSTALL 0x0F
#define REG_PGA_10DB_BOOST_BOOSTNONE 0x00

#define REG_POP_SUPRESS 0x0029 //leave as default, pop supression is on
#define REG_POP_SUPRESS_HDPHNON ((1 << 5) | (1 << 4))
#define REG_POP_SUPRESS_HDPHNOFF ((0 << 5) | (0 << 4))
#define REG_POP_SUPRESS_PGAON 0x7
#define REG_POP_SUPRESS_PGAOFF 0x0
#define REG_POP_SUPRESS_ALLON (REG_POP_SUPRESS_HDPHNON | REG_POP_SUPRESS_PGAON)
#define REG_POP_SUPRESS_ALLOFF (REG_POP_SUPRESS_HDPHNOFF | REG_POP_SUPRESS_PGAOFF)

#define REG_DSPBYPASS_PATH 0x002A //connect ADC to DAC 0 and 1 for bypass mode
#define REG_DSPBYPASS_PATH_DEFAULT 0x03

#define REG_DSPBYPASSGAIN_0 0x002B //set gain for DSP Bypass mode
#define REG_DSPBYPASSGAIN_1 0x002C //leave as default for now

#define REG_MICBIAS_01 0x002D //off for now, leave as default
//may want to use with external mics

#define REG_DAC_CTRL_01 0x002E  //set phase, enable DAC, unmute it
#define REG_DAC_CTRL_01_INVERTPHASE (1 << 5)
#define REG_DAC_CTRL_01_NORMALPHASE (0 << 5)
#define REG_DAC_CTRL_01_MUTE ((1 << 4) | (1 << 3))
#define REG_DAC_CTRL_01_UNMUTE ((0 << 4) | (0 << 3))
#define REG_DAC_CTRL_01_ENABLE ((1 << 1) | (1 << 0))
#define REG_DAC_CTRL_01_DISABLE ((0 << 1) | (0 << 0))
#define REG_DAC_CTRL_01_DEFAULT (REG_DAC_CTRL_01_NORMALPHASE | REG_DAC_CTRL_01_UNMUTE | REG_DAC_CTRL_01_ENABLE)

#define REG_DAC0_VOL 0x002F //0b00000000 = 0dBFS
#define REG_DAC1_VOL 0x0030 //0b11111111 = -95.625dbFS
//leave as default 0dBFS for now, unless clipping of output

#define REG_HDPHN_MUTE 0x0031 //defaults to muted
#define REG_HDPHN_MUTE_UNMUTE 0x0A //single ended mode 0b 0000 1010, leave N side muted, P side unmuted
#define REG_HDPHN_MUTE_MUTE  0x0F

#define REG_SDATA_MODE0 0x0032 //leave as default

#define REG_SDATA_MODE1 0x0033 //tristate outputs
#define REG_SDATA_MODE1_DEFAULT 0x80

#define REG_SDATA_TDM 0x0034 //disable all TDM data
#define REG_SDATA_TDM_DEFAULT 0xFF

#define REG_PDM_ENABLE 0x0036 //leave as default, disabled
#define REG_PDM_PATTERN 0x0037 //leave as default

#define REG_MP0_MODE 0x0038  //default serial input 0
#define REG_MP1_MODE 0x0039  //default push vol up
#define REG_MP2_MODE 0x003A  //default bit clock
#define REG_MP3_MODE 0x003B  //default LR clock
#define REG_MP4_MODE 0x003C  //default dig mic input 0/1
#define REG_MP5_MODE 0x003D  //default dig mic input 2/3
#define REG_MP6_MODE 0x003E  //default push vol down
//leave as defaults for now, change soon

#define REG_MP_MODE_MUTEDACS 0x0A
#define REG_MP_MODE_BANKSWITCH 0x0B
#define REG_MP_MODE_BYPASSENABLE 0x0F
#define REG_MP_MODE_VOLUP 0x10
#define REG_MP_MODE_VOLDOWN 0x11


#define REG_PB_VOL_SET 0x003F
#define REG_PB_VOL_SET_INIT0DB (0b00000 << 3)
#define REG_PB_VOL_SET_INITM6DB (0b00100 << 3)
#define REG_PB_VOL_SET_INITM12DB (0b01000 << 3)
#define REG_PB_VOL_SET_INITM20DB (0b01101 << 3)
#define REG_PB_VOL_SET_INITM30DB (0b10100 << 3)
#define REG_PB_VOL_SET_INITM45DB (0b11111 << 3)
#define REG_PB_VOL_SET_HOLDTIME150MS 0x0
#define REG_PB_VOL_SET_HOLDTIME300MS 0x1
#define REG_PB_VOL_SET_HOLDTIME450MS 0x2
#define REG_PB_VOL_SET_HOLDTIME600MS 0x3
#define REG_PB_VOL_SET_HOLDTIME900MS 0x4
#define REG_PB_VOL_SET_HOLDTIME1200MS 0x5
#define REG_PB_VOL_SET_DEFAULT (REG_PB_VOL_SET_INITM20DB | REG_PB_VOL_SET_HOLDTIME300MS)
//defaults to 0dB and 150ms, but if not written starts at -96dB
//must be written before PB_VOL_CTRL

#define REG_PB_VOL_CTRL 0x0040
#define REG_PB_VOL_CTRL_GAINSTEP0P3 ((0 << 7) | (0 << 6))
#define REG_PB_VOL_CTRL_GAINSTEP1P5 ((0 << 7) | (1 << 6))
#define REG_PB_VOL_CTRL_GAINSTEP3 ((1 << 7) | (0 << 6))
#define REG_PB_VOL_CTRL_GAINSTEP4P5 ((1 << 7) | (1 << 6))
#define REG_PB_VOL_CTRL_RAMPSPEED60DB (0x0 << 3)
#define REG_PB_VOL_CTRL_RAMPSPEED36DB (0x3 << 3)
#define REG_PB_VOL_CTRL_RAMPSPEED18DB (0x5 << 3)
#define REG_PB_VOL_CTRL_RAMPSPEED6DB (0x7 << 3)
#define REG_PB_VOL_CTRL_ADCS 0x2
#define REG_PB_VOL_CTRL_DACS 0x3
#define REG_PB_VOL_CTRL_NONE 0x7
#define REG_PB_VOL_CTRL_DEFAULT (REG_PB_VOL_CTRL_GAINSTEP3 | REG_PB_VOL_CTRL_RAMPSPEED36DB | REG_PB_VOL_CTRL_NONE)
//set gainstep, rampspeed, and converters to be controlled
//leave as default, vol up/down control no converters, 3db step, 60db/sec rmap

#define REG_DEBOUNCE_MODE 0x0041
#define REG_DEBOUNCE_MODE_300US 0x0
#define REG_DEBOUNCE_MODE_600US 0x1
#define REG_DEBOUNCE_MODE_900US 0x2
#define REG_DEBOUNCE_MODE_5MS 0x3
#define REG_DEBOUNCE_MODE_10MS 0x4
#define REG_DEBOUNCE_MODE_20MS 0x5
#define REG_DEBOUNCE_MODE_40MS 0x6
#define REG_DEBOUNCE_MODE_OFF 0x7
//leave as default, 20ms debounce

#define REG_HDPHN_MODE 0x0043
#define REG_HDPHN_MODE_HDPHNENABLE ((1 << 5) | (1 << 4))
#define REG_HDPHN_MODE_LINEOUTENABLE ((0 << 5) | (0 << 4))
#define REG_HDPHN_MODE_POWERDOWNALL 0xF
#define REG_HDPHN_MODE_ENABLEP 0xA //this is what we want for gnd-centered hdphns
#define REG_HDPHN_MODE_ENABLEALL 0x0
#define REG_HDPHN_MODE_DEFAULT (REG_HDPHN_MODE_HDPHNENABLE | REG_HDPHN_MODE_ENABLEP)
//after enabling this, wait 6ms to unmute (REG_HDPHN_MUTE)

#define REG_DECIM_PWR_MODE 0x0044
#define REG_DECIM_PWR_MODE_PWRADC01 0x3
#define REG_DECIM_PWR_MODE_PWRALLADC 0xF
#define REG_DECIM_PWR_MODE_PWRASRC01 (0x3 << 4)
#define REG_DECIM_PWR_MODE_PWRALLASRC (0xF << 4)
#define REG_DECIM_PWR_MODE_DEFAULT (REG_DECIM_PWR_MODE_PWRADC01)
//power to ASRC and ADC filters

#define REG_DACMOD_ASRCINTERP_MODE 0x0045
//leave as default, powered down DAC modulator and ASRC Interpolator

#define REG_BIAS_CTRL0 0x0046
#define REG_BIAS_CTRL0_NORMAL 0x00
#define REG_BIAS_CTRL0_ENERGYSAVE 0xFF
#define REG_BIAS_CTRL0_ENHANCEDPERF 0xAA
//not sure which of these can be disabled, mic bias?
//leave as default for now (normal)

#define REG_BIAS_CTRL1 0x0047
#define REG_BIAS_CTRL1_NORMAL 0x00
#define REG_BIAS_CTRL1_ENERGYSAVE 0x3F
#define REG_BIAS_CTRL1_ENHANCEDPERF 0x2A
#define REG_BIAS_CTRL1_BIASON (0 << 6)
#define REG_BIAS_CTRL1_BIASOFF (1 << 6)
//not sure which of these can be disabled, mic bias?
//leave as default for now (normal, bias on)

#define REG_DIGPULLUP0 0x0048
#define REG_DIGPULLUP1 0x0049
#define REG_DIGPULLDOWN0 0x004A
#define REG_DIGPULLDOWN1 0x004B
//leave as default, all digital pull-ups/pull-downs disabled

#define REG_DIGDRIVESTRENGTH0 0x004C
//leave as default, low drive strength MP pins

#define REG_DIGDRIVESTRENGTH1 0x004D
#define REG_DIGDRIVESTRENGTH1_I2CHIGHSTRENGTH ((1 << 3) | (1 << 2))
//leave as default, can use high drive strength if required for I2C


/////////////////////////////////////////////////////////////////
//SIGNAL PROCESSING SETTINGS
/////////////////////////////////////////////////////////////////

//BANK A
//B0/MAX GAIN   B1/MIN GAIN   	B2/ATTACK   	A1/DECAY   		A2/THRESH
// 0x00E0       0x0100			0x0120			0x0140			0x0160
// 0x00E1		0x0101			0x0121			0x0141			0x0161
// ...			...				...				...				...
// 0x00FF		0x011F			0x013F			0x015F			0x017F

//BANK B
//B0/MAX GAIN   B1/MIN GAIN   	B2/ATTACK   	A1/DECAY   		A2/THRESH
// 0x0180       0x01A0			0x01C0			0x01E0			0x0200
// 0x0181		0x01A1			0x01C1			0x01E1			0x0201
// ...			...				...				...				...
// 0x019F		0x01BF			0x01DF			0x01FF			0x021F

//B0,B1,B2 5.27
//A1 2.27 (sign extended)
//A2 1.27 (sign extended)
//Max Gain, Min Gain, Thresh 2.23
//Attack, Decay Time 24.0



//MEMORY LOCATIONS

uint16_t param_mem[40]={ 0x00E0,
						0x0100,
						0x0120,
						0x0140,
						0x0160,
						0x0180,
						0x01A0,
						0x01C0,
						0x01E0,
						0x0200,
						0x00E2,
						0x0102,
						0x0122,
						0x0142,
						0x0162,
						0x0182,
						0x01A2,
						0x01C2,
						0x01E2,
						0x0202,
						0x00E1,
						0x0101,
						0x0121,
						0x0141,
						0x0161,
						0x0181,
						0x01A1,
						0x01C1,
						0x01E1,
						0x0201,
						0x00E3,
						0x0103,
						0x0123,
						0x0143,
						0x0163,
						0x0183,
						0x01A3,
						0x01C3,
						0x01E3,
						0x0203};

// PARAMS THAT CORRESPOND TO THESE LOCATIONS

uint8_t param_data[160] = { 0x08, 0x00, 0x00, 0x00,
						0x90, 0x00, 0x00, 0x00,
						0x08, 0x00, 0x00, 0x00,
						0xFF, 0x42, 0x90, 0x36,
						0x07, 0x4A, 0xF2, 0x0E,
						0x08, 0x00, 0x00, 0x00,
						0x10, 0x00, 0x00, 0x00,
						0x08, 0x00, 0x00, 0x00,
						0xFF, 0x42, 0x90, 0x36,
						0x07, 0x4A, 0xF2, 0x0E,
						0x08, 0x00, 0x00, 0x00,
						0x10, 0x00, 0x00, 0x00,
						0x08, 0x00, 0x00, 0x00,
						0xFF, 0x42, 0x90, 0x36,
						0x07, 0x4A, 0xF2, 0x0E,
						0x08, 0x00, 0x00, 0x00,
						0x90, 0x00, 0x00, 0x00,
						0x08, 0x00, 0x00, 0x00,
						0xFF, 0x42, 0x90, 0x36,
						0x07, 0x4A, 0xF2, 0x0E,
						0x00, 0xA1, 0x24, 0x78,
						0x00, 0x0C, 0xCC, 0xCD,
						0x00, 0x00, 0x03, 0xE8,
						0x00, 0x00, 0x03, 0xE8,
						0x00, 0x28, 0x7A, 0x27,
						0x00, 0x80, 0x00, 0x00,
						0x00, 0x01, 0x47, 0xAE,
						0x00, 0x00, 0x01, 0x2C,
						0x00, 0x00, 0x01, 0x2C,
						0x00, 0x0C, 0xCC, 0xCD,
						0x00, 0xA1, 0x24, 0x78,
						0x00, 0x0C, 0xCC, 0xCD,
						0x00, 0x00, 0x03, 0xE8,
						0x00, 0x00, 0x03, 0xE8,
						0x00, 0x28, 0x7A, 0x27,
						0x00, 0x80, 0x00, 0x00,
						0x00, 0x01, 0x47, 0xAE,
						0x00, 0x00, 0x01, 0x2C,
						0x00, 0x00, 0x01, 0x2C,
						0x00, 0x0C, 0xCC, 0xCD};



////////////////////////////////////////////////////////////////////
//PROGRAM MEMORY
////////////////////////////////////////////////////////////////////
//0x0080 - 0x009F  (32 instructions)
//

/* op codes from SigmaStudio to create a system that:

passes AIN0/1 through a parameteric EQ, limiter, and out the other end
passes AIN0/1 to DBREG0/1

0010000000
0110000100
1000010000
0110001000
0110000000
1100000000
0110001100
1100000001
0110001101
0010000001
0110000100
1000110000
0110001001
0110000001

PEQ1FilterChan1     	Param EQ1           	0
Limit17721          	Limit1              	1
PEQ1FilterChan2     	Param EQ2           	2
Limit17722          	Limit2              	3
*/


//Memory Locations
uint16_t prog_mem[15] = { 0x0080,
						0x0081,
						0x0082,
						0x0083,
						0x0084,
						0x0085,
						0x0086,
						0x0087,
						0x0088,
						0x0089,
						0x008A,
						0x008B,
						0x008C,
						0x008D,
						0x008E};

//Hex opcodes
uint8_t prog_data[30] = { 0x00, 0x80,
						0x01, 0x84,
						0x02, 0x10,
						0x01, 0x88,
						0x01, 0x80,
						0x03, 0x00,
						0x01, 0x8C,
						0x03, 0x01,
						0x01, 0x8D,
						0x00, 0x81,
						0x01, 0x84,
						0x02, 0x30,
						0x01, 0x89,
						0x01, 0x81};










static I2CDriver *driver;


static void audio_set(uint16_t address, uint8_t val) {
//audio set for control registers

  uint8_t tx[3];
  tx[0] = (address >> 8);
  tx[1] = (address & 0xFF);
  tx[2] = val;

  i2cMasterTransmitTimeout(driver, audioAddr, 
                           tx, sizeof(tx),
                           NULL, 0,
                           TIME_INFINITE);
}



static void audio_set_p(uint16_t address, uint8_t val[], int valSize) {
//audio set for program and parameter registers

  uint8_t tx[6];
  tx[0] = (address >> 8);
  tx[1] = (address & 0xFF);

  memcpy(tx + 2*sizeof(uint8_t), val, valSize*sizeof(uint8_t));
  
  i2cMasterTransmitTimeout(driver, audioAddr, 
                           tx, (2+valSize),
                           NULL, 0,
                           TIME_INFINITE);
}


static uint8_t audio_get(uint16_t full_reg) {
  
  uint8_t reg[2];
  reg[0] = (full_reg >> 8);
  reg[1] = (full_reg & 0xFF);

  uint8_t val;

  i2cMasterTransmitTimeout(driver, audioAddr,
                           &reg, sizeof(reg),
                           &val, 1,
                           TIME_INFINITE);
  return val;
}


static void initializeAudioParams(){
   //pass arrays of params into memory

  for (int i=0;i<sizeof(param_mem);i++){
	  audio_set_p(param_mem[i], &param_data[LENGTH_REG_PARAM*i], LENGTH_REG_PARAM);
  }

} 


static void initializeAudioProgram(){
   //pass arrays of program data into memory
   
  for (int i=0;i<sizeof(prog_mem);i++){
	  audio_set_p(prog_mem[i], &prog_data[LENGTH_REG_PROGM*i], LENGTH_REG_PROGM);
  }

} 


void audioStop(void) {
 
  i2cAcquireBus(driver);
  
  audio_set(REG_HDPHN_MUTE, REG_HDPHN_MUTE_MUTE); //mute headphones
  //ADD A BUNCH MORE TO POWER DOWN CHIP
  
  i2cReleaseBus(driver);

}


void audioStart(I2CDriver *i2cp) {

  driver = i2cp;
  i2cAcquireBus(driver);

  //startup I2C register setting
  
  /* //zero out all PLL registers
  audio_set(REG_CLK_CTRL, REG_CLK_CTRL_SETUP);
  audio_set(REG_PPL_CTRL0, REG_PLL_CTRL_DEFAULT);
  audio_set(REG_PPL_CTRL1, REG_PLL_CTRL_DEFAULT);
  audio_set(REG_PPL_CTRL2, REG_PLL_CTRL_DEFAULT);
  audio_set(REG_PPL_CTRL3, REG_PLL_CTRL_DEFAULT);
  audio_set(REG_PPL_CTRL4, REG_PLL_CTRL_DEFAULT);
  */
  
  audio_set(REG_CLK_CTRL, REG_CLK_CTRL_DEFAULT); //init/start core clock
  audio_set(REG_CLKOUT_SEL, REG_CLKOUT_DEFAULT); //set clkout off
 
  //audio_set(REG_REGULATOR, REG_REGULATOR_DEFAULT); //set regulator to 1.2V/active
  //audio_set(REG_CORE_ENABLE, REG_CORE_ENABLE_DEFAULT); // turn on DSP clock and limiter functionality
  
  //audio_set(REG_COREIN_01, REG_COREIN_01_DEFAULT); //map inputs to core channels
  //audio_set(REG_COREIN_23, REG_COREIN_23_DEFAULT); //map extra inputs to extra core channels
  //audio_set(REG_DAC_SOURCE_01, REG_DAC_SOURCE_01_DEFAULT); //map core channel to DAC
  
  //audio_set(REG_ASRC_MODE, REG_ASRC_MODE_DEFAULT); //disable ASRCs
  
  initializeAudioProgram();
  initializeAudioParams();
  
  audio_set(REG_DSPBYPASS_PATH, REG_DSPBYPASS_PATH_DEFAULT); //define default bypass path
  
  audio_set(REG_PGAAIN0_CTRL, REG_PGA_CTRL_DEFAULT); //put PGA in mic mode
  audio_set(REG_PGAAIN1_CTRL, REG_PGA_CTRL_DEFAULT); //put PGA in mic mode
  
  audio_set(REG_SDATA_MODE1, REG_SDATA_MODE1_DEFAULT); //tristate unused TDM output
  audio_set(REG_SDATA_TDM, REG_SDATA_TDM_DEFAULT); //disable all TDM outputs
  
  audio_set(REG_ADC_CTRL1_01, REG_ADC_CTRL1_01_DEFAULT);  //enable ADC and a 8Hz HPF for channels 0/1
  audio_set(REG_DECIM_PWR_MODE, REG_DECIM_PWR_MODE_DEFAULT); //power on ADC filter clocks 0/1
  audio_set(REG_ADC_CTRL_01, REG_ADC_CTRL_01_DEFAULT); //unmute ADC, set sample rate
  
  audio_set(REG_CORE_CTRL, REG_CORE_CTRL_DEFAULT); //init/start core processing audio
  
  audio_set(REG_DAC_CTRL_01, REG_DAC_CTRL_01_DEFAULT); //unmute DAC (enable it and set phase)
  audio_set(REG_HDPHN_MODE, REG_HDPHN_MODE_DEFAULT);//after enabling this, wait 6ms to unmute hdphns(REG_HDPHN_MUTE)
  
  chThdSleep(MS2ST(6)+1); //delay 6 ms
  
  audio_set(REG_HDPHN_MUTE, REG_HDPHN_MUTE_UNMUTE); //unmute headphones
    
  i2cReleaseBus(driver);
  
}





msg_t audioPoll(struct audio_data *data) {
  uint8_t tx[6]={0x00, 0x0C, 0x00, 0x0D, 0x00, 0x0E};
  uint8_t rx[3];


  i2cAcquireBus(driver);

  //pull REG_DBREG0
  i2cMasterTransmitTimeout(driver, audioAddr,
                           &tx[0], 2*sizeof(uint8_t),
                           &rx[0], sizeof(uint8_t),
                           TIME_INFINITE);

  //pull REG_DBREG1
  i2cMasterTransmitTimeout(driver, audioAddr,
                           &tx[2], 2*sizeof(uint8_t),
                           &rx[1], sizeof(uint8_t),
                           TIME_INFINITE);
  
  //pull REG_DBREG2
  i2cMasterTransmitTimeout(driver, audioAddr,
                           &tx[4], 2*sizeof(uint8_t),
                           &rx[2], sizeof(uint8_t),
                           TIME_INFINITE);

  i2cReleaseBus(driver);

  data->x = rx[0];//reg0
  data->y = rx[1];//reg1
  data->z = rx[2];//reg2

  return MSG_OK;
}
