#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_letimer.h"
#include "em_system.h"
#include "em_device.h"
#include "em_int.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "em_dma.h"
#include "em_lesense.h"
#include "em_rtc.h"
#include "dmactrl.h"
#include "em_i2c.h"
#include "em_leuart.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <malloc.h>
#include <stdlib.h>

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

#define energyMode EM2				// Setting the energy mode

#define ON_TIME 0.004
#define OFF_TIME 3				//given specification (letimer period) : 0.625

#define CALIBRATION 1

int mod = 1;
uint32_t ULFRCO_COUNTS= 1000;
uint32_t LFXO_COUNTS = 32768;

uint32_t REFERENCE_LEVEL=2;  		// Ambient sensor reference level

uint32_t lfxo_count=0;
uint32_t ulfrco_count=0;


/*Defining GPIO port and pin*/
#define LEDPORT gpioPortE
#define LEDPIN 2
#define LEDPORT_TEMPERATURE gpioPortE
#define LEDPIN_TEMPERATURE 3


#define I2C0_SDA_PORT gpioPortD				//SDA port
#define I2C0_SDA_PIN 6
#define I2C0_SCL_PORT gpioPortD				//SCL pin
#define I2C0_SCL_PIN 7

#define I2C1_SDA_PORT gpioPortC				//SDA port
#define I2C1_SDA_PIN 4
#define I2C1_SCL_PORT gpioPortC				//SCL pin
#define I2C1_SCL_PIN 5


#define I2C1_INT_PORT gpioPortD				/* Interrupt */
#define I2C1_INT_PIN 1
#define I2C1_VDD_PORT gpioPortD				/* Vdd in I2C1 */
#define I2C1_VDD_PIN 0

#define I2C0_VDD_PORT gpioPortC
#define I2C0_VDD_PIN 3


#define DMA_CHANNEL_ADC  0
#define TEMP_LOW 15              // Lower temperture limit
#define TEMP_HIGH 35             // Higher temperature limit
#define DMA_ON   1               // Switch to turn on or off the dma


// I2C addresses for the light sensor
#define SLAVE_ADDRESS    	    0x39
#define COMMAND    	  		    0x80    // Command register
#define CONTROL  				      0x00
#define TIMING  				      0x01
#define THRESHLOWLOW  			  0x02
#define THRESHLOWHIGH  			  0x03
#define THRESHHIGHLOW  			  0x04
#define THRESHHIGHHIGH  		  0x05
#define INTERRUPT  				    0x06
#define CRC  					        0x08
#define ID  					        0x0A
#define DATA0LOW  				    0x0C
#define DATA0HIGH  				    0x0D
#define DATA1LOW  				    0x0E
#define DATA1HIGH  				    0x0F

#define PERSISTENCE 			0x04
#define INTR					    0x01		// INTR bits in the interrupt register of TSL2561
#define INTEG					    0x02		// integration time is set to 101 ms


// I2C addresses for bme sensor
#define BME_SLAVE_ADDR				  0x76

#define BME_CHIP_ID_REGISTER		0xD0
#define BME_CHIP_ID_DEFAULT			0x60
#define BME_CTRL_HUM				    0xF2
#define BME_CTRL_MEAS				    0xF4
#define BME_CONFIG					    0xF5
#define BME_MODE_NORMAL				  0x03 	//reads sensors at set interval
#define BME_MODE_FORCED				  0x01 	//reads sensors once when you write this register


// for BME sensor
uint8_t osrs_t = 1;             //Temperature oversampling x 1
uint8_t osrs_p = 1;             //Pressure oversampling x 1
uint8_t osrs_h = 1;             //Humidity oversampling x 1
uint8_t time_standby = 4;       //Tstandby, 5=1000ms, 4=500ms
uint8_t iir_filter = 0;         //Filter off
uint8_t spi_enable = 0;         //3-wire SPI Disable
uint8_t msb, lsb, xsb;

//calibration registers for the BME sensor
uint16_t calib_dig_T1;
int16_t calib_dig_T2;
int16_t calib_dig_T3;
uint16_t calib_dig_P1;
int16_t calib_dig_P2;
int16_t calib_dig_P3;
int16_t calib_dig_P4;
int16_t calib_dig_P5;
int16_t calib_dig_P6;
int16_t calib_dig_P7;
int16_t calib_dig_P8;
int16_t calib_dig_P9;
int8_t  calib_dig_H1;
int16_t calib_dig_H2;
int8_t  calib_dig_H3;
int16_t calib_dig_H4;
int16_t calib_dig_H5;
int8_t  calib_dig_H6;

uint8_t BME_OperationMode = BME_MODE_FORCED;
uint8_t weather_code = 0;

unsigned long int raw_hum, raw_temp, raw_pres;
signed long int t_fine;
signed long int temp_act;
unsigned long int press_act, hum_act;
float actual_temperature, actual_humidity, altitude;
float actual_pressure = 0.0;

volatile int a = 0;
char text[11];

#define DISABLE_PASSIVE_LIGHT_SENSOR 1 // if the value is 1, active light sensor is activated

unsigned int lowestEnergyMode[5];

DMA_CB_TypeDef cb;		/* DMA callback structure */
bool transferActive;	/* Transfer Flag */

/* ADC Transfer Data */
#define ADCSAMPLES                500
volatile uint16_t ramBufferAdcData[ADCSAMPLES];
#define ADCSAMPLESPERSEC          100000

int dmaCount = 0;       //Number of counts using dma
int adcCount = 0;       // Number of counts without dma

long sum = 0;         // sum of adc results with dma
float average = 0.0; // average of 1000 transfers


long sum1 = 0;         // sum of adc results without dma
float average1 = 0.0; // average of 1000 transfers
uint8_t integer_average = 0;  //typecasting the average to extract integer part
uint16_t average_fractional = 0;
uint8_t average_fractional_2 = 0;	//typecasting the average to extract fractional part

uint8_t integer_average_press1 = 0;	//for extracting pressure
uint8_t integer_average_press2 = 0;	//for extracting pressure
uint16_t average_fractional_press = 0;
uint8_t average_fractional__press2 = 0;	//typecasting the average to extract fractional part


//LESENSE parameters
/** Scan frequency for LESENSE, how often all the pads are scanned in a second. */
#define LESENSE_SCAN_FREQUENCY          2

/** Sample delay, how long the rc-oscillations are sampled. */
#define SAMPLE_DELAY                   30

/** Validate count is the number of consecutive scan-cycles a button needs to */
/** be in the changed state before an actual button press or release is acknowledged. */
#define VALIDATE_CNT                   10

/** Number of calibration events used to calculate threshold. */
#define NUMBER_OF_CALIBRATION_VALUES    10

/** Interval between calibration, in seconds. */
#define CALIBRATION_INTERVAL            5

/* RTC nominal frequency */
#define RTC_FREQ               32768

/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS    16

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT         gpioPortC

static volatile uint16_t calibration_value[NUM_LESENSE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static volatile uint16_t buttons_pressed;
static volatile uint16_t channel_max_value[NUM_LESENSE_CHANNELS];
static volatile uint16_t channel_min_value[NUM_LESENSE_CHANNELS];

static volatile uint8_t channel_no = 0;

static uint16_t channels_used_mask;
static uint8_t num_channels_used;
static float channel_threshold_percent[NUM_LESENSE_CHANNELS];

int adc_sensor_count = 0;
uint8_t data0 = 0;
uint8_t data1 = 0;

uint8_t mod_check = 0;
uint16_t x = 0;
uint8_t neg_value = 'b';	//variable indicating whether the average is positive('b') or negative('a')


/*Initialising all the setup routines*/

void sleep(void);
void blockSleep(unsigned int minmode);
void unblockSleep(unsigned int minmode);
void CMU_setup(void);
void GPIO_setup(void);
void ACMP_setup(void);
void TIMER_Setup(void);
void LETIMER_Setup(void);
void LED_Activate(bool state);
void Calibration(void);
float convertToCelsius(float sample);
void setupAdc(void);
void setupDma(void);
void setupI2C0(void);
void setupI2C1(void);
void I2C0_Write(uint8_t addr, uint8_t data);
void I2C1_Write(uint8_t addr, uint8_t data);
void I2C0_sensor(void);
void I2C1_sensor(void);
uint16_t I2C0_Read(uint8_t addr);
uint16_t I2C1_Read(uint8_t addr);
void stabilisation(void);
void power_up(void);
void power_down(void);
void ftoa(double number,char * string, int precision);
void initLeuart(void);

bool setup_BME(uint8_t operationMode);
bool BME_verify_ChipId(void);
void BME_writeConfig(void);
void BME_readCalibrationRegisters(void);
void BME_readSensorData(void);
void BME_readActualSensorData(void);

unsigned long int BME_getTemperatureRaw(void);
unsigned long int BME_getPressureRaw(void);
unsigned long int BME_getHumidityRaw(void);

signed long int BME_getTemperature(void);
unsigned long int BME_getPressure(void);
unsigned long int BME_getHumidity(void);

signed long int BME_calibration_Temp(signed long int adc_T);
unsigned long int BME_calibration_Press(signed long int adc_P);
unsigned long int BME_calibration_Hum(signed long int adc_H);

void LETOUCH_Init(float sensitivity[]);
uint16_t LETOUCH_GetChannelsTouched(void);
uint16_t LETOUCH_GetChannelMaxValue(uint8_t channel);
uint16_t LETOUCH_GetChannelMinValue(uint8_t channel);
void LETOUCH_Calibration(void);
static void LETOUCH_setupACMP(void);
static void LETOUCH_setupLESENSE(void);
static void LETOUCH_setupGPIO(void);
static void LETOUCH_setupRTC(void);
static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N);
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N);


/****************************************************************************/
/**
* @file sleep.c
******************************************************************************
*
* @section License
* <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
******************************************************************************
*
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
* DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
* obligation to support this Software. Silicon Labs is providing the
* Software "AS IS", with no express or implied warranties of any kind,
* including, but not limited to, any implied warranties of merchantability
* or fitness for any particular purpose or warranties against infringement
* of any proprietary rights of a third party.
*
* Silicon Labs will not be liable for any consequential, incidental, or
* special damages, or any other relief, or for any claim by any third party,
* arising from your use of this Software.
*
******************************************************************************/


/*This code is originally Silicon Labs and copyrighted by Silicon Labs' and Silicon Labs grants permission to
 * anyone to use the software for any purpose, including commercial applications, and to alter it and redistribute
 * subject that the origins are not altered, misrepresented and this notice cannot be altered or removed from any
 * source distribution
 */


/*Sleep Routine*/
void sleep(void){
	if(lowestEnergyMode[0]>0){
		return;								/*Blocked all energy modes below EM0*/
	}
	else if(lowestEnergyMode[1]>0){
		EMU_EnterEM1();						/*Blocked all energy modes below EM1 and Enter EM1*/
	}
	else if(lowestEnergyMode[2]>0){
		EMU_EnterEM2(true);					/*Blocked all energy modes below EM2 and Enter EM2*/
	}
	else if(lowestEnergyMode[3]>0){
		EMU_EnterEM3(true);					/*Blocked all energy modes below EM3 and Enter EM3*/
	}
	else{
		EMU_EnterEM4();						/*Enter EM4*/
	}
}

// Block sleep routine
void blockSleep(unsigned int minmode){				/*Setting the appropriate Energy mode and blocking the modes under that*/
	INT_Disable();
	lowestEnergyMode[minmode]++;
	INT_Enable();
}


//Unblock sleep routine
void unblockSleep(unsigned int minmode){			/*Releasing the blocks under the given energy mode*/
	INT_Disable();
	if(lowestEnergyMode[minmode]>0){
		lowestEnergyMode[minmode]--;
	}
	else{
		lowestEnergyMode[minmode]=0;
	}
	INT_Enable();
}

void ftoa(double number,char * string, int precision)
{
    sprintf (string,"%d.%02u", (int) number, (int) ((number - (int) number ) * precision) );
}


/**************************************************************************//**
DMA Callback routine
 *****************************************************************************/
void callback(unsigned int channel, bool primary, void *user)
{
	  (void) channel;
	  (void) primary;
  	(void) user;

  	INT_Disable();

  	//The rest of the function runs when the dmaCount is equal to 500, i.e., the 500 samples are taken
	  ADC0 -> CMD |= ADC_CMD_SINGLESTOP;
	  unblockSleep(EM1);

	  while(dmaCount<ADCSAMPLES){
		  sum = sum + ramBufferAdcData[dmaCount];
		  dmaCount ++;
	  }

	  average = sum/ADCSAMPLES;
	  average = convertToCelsius(average);             //Converting to temperature
    /*if((average > TEMP_LOW) & (average < TEMP_HIGH) ){
		  GPIO_PinOutClear(LEDPORT_TEMPERATURE,LEDPIN_TEMPERATURE);
	  }
	  else
		  GPIO_PinOutSet(LEDPORT_TEMPERATURE,LEDPIN_TEMPERATURE);*/

	  if(average<0){
		  neg_value = 'a';
		  average = average * (-1);
	  }
	  else
		  neg_value = 'b';

	  integer_average = (uint8_t)average;
	  initLeuart();		//calling the LEUART initialization function

	  /* Clearing flag to indicate that transfer is complete */
	  transferActive = false;
	  sum = 0;
	  dmaCount = 0;

	  INT_Enable();

}


/**************************************************************************//**
This is the function to set up the dma.
The logic for this function has been built using Silicon Labs's example Dma code.
 *****************************************************************************/
void setupDma(void)
{
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Setting up call-back function */
  cb.cbFunc  = callback;
  cb.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;

  DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);

}


/**************************************************************************//**
 *This function sets up the ADC module in the single mode.
 *****************************************************************************/
void setupAdc(void)
{
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;

  /* Configure ADC single mode to sample Ref/2 */
  adcInit.prescale = ADC_PrescaleCalc( ADCSAMPLESPERSEC * 13, 0); /* Set frequency of 13*10000 = 130000MHz */
  ADC_Init(ADC0, &adcInit);

  adcInitSingle.input     =  adcSingleInputTemp;   /* temperature inout */
  ADC_InitSingle(ADC0, &adcInitSingle);

	ADC0->SINGLECTRL &= 0xFFFFFFFE;					/*Setting the REP mode ON */
	ADC0->SINGLECTRL |= 0x00000001;

	if(DMA_ON == 0){
			  ADC0 -> IEN = ADC_IEN_SINGLE;       // Enabling the single bit in the interrupt field
			  NVIC_EnableIRQ(ADC0_IRQn);
			  ADC0->SINGLECTRL &= 0xFFFFFFFE;
        ADC0->SINGLECTRL |= 0x00000000;      // Disabling the REP mode
	}
}


/*Clock Setup Routine*/
void CMU_setup(){
	 /* Ensure core frequency has been updated */
	  SystemCoreClockUpdate();

	/*Set up the high frequency oscillator*/
	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
	CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFRCO);		/*Set HFRCO as HF system clock*/
	CMU_OscillatorEnable(cmuOsc_HFXO,false,false);			/*Disabling HF crystal oscillator in order to save energy*/
	CMU_ClockEnable(cmuClock_HFPER,true);				      	/*Enabling the HF peripheral clock*/

	/*Enable Low Frequency oscillator for low frequency peripherals*/
	CMU_OscillatorEnable(cmuOsc_LFXO,true,true);

	if(energyMode==EM3){
		CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);		/*Selecting ULFRCO clock*/
		CMU_OscillatorEnable(cmuOsc_LFXO,false,false);			/* Disabling LFXO oscillator*/
	}
	else{
		CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);		/*Selecting the LFXO clock*/
	}

	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);					/*Enabling clock for LETIMER*/
	CMU_ClockEnable(cmuClock_GPIO, true);						/*Enabling clock for GPIO*/

	/* ACMP */
	CMU_ClockEnable(cmuClock_ACMP0, true);
	CMU_ClockEnable(cmuClock_ACMP1, true);
	CMU_ClockEnable(cmuClock_LESENSE, true);
	CMU_ClockEnable(cmuClock_RTC, true);
	CMU_ClockEnable(cmuClock_TIMER0,true);						/*Enabling clock for TIMER0*/
	CMU_ClockEnable(cmuClock_TIMER1,true);						/*Enabling clock for TIMER1*/
	CMU_ClockEnable(cmuClock_ADC0,true);					  	/*Enabling clock for ADC0*/
	CMU_ClockEnable(cmuClock_DMA, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_I2C1, true);
}


// Timer setup function. This is needed only for the calibration routine.
void TIMER_Setup(void){
	  int intflags0;
	  int intflags1;

	  TIMER_Init_TypeDef timerInit0 =
	  {
	    .enable     = false,
	    .debugRun   = true,
	    .prescale   = timerPrescale1,
	    .clkSel     = timerClkSelHFPerClk,
	    .fallAction = timerInputActionNone,
	    .riseAction = timerInputActionNone,
	    .mode       = timerModeUp,
	    .dmaClrAct  = false,
	    .quadModeX4 = false,
	    .oneShot    = false,
	    .sync       = false,
	  };
	  /*Initializing TIMER0*/
	  TIMER_Init(TIMER0, &timerInit0);

	  /* Select TIMER0 parameters */
	  TIMER_Init_TypeDef timerInit1 =
	  	  {
	  	    .enable     = false,
	  	    .debugRun   = true,
	  	    .prescale   = timerPrescale1,
	  	    .clkSel     = timerClkSelCascade,
	  	    .fallAction = timerInputActionNone,
	  	    .riseAction = timerInputActionNone,
	  	    .mode       = timerModeUp,
	  	    .dmaClrAct  = false,
	  	    .quadModeX4 = false,
	  	    .oneShot    = false,
	  	    .sync       = true,
	  	  };
        
	  /*Initializing TIMER1*/
	  TIMER_Init(TIMER1, &timerInit1);

	  intflags0 = LETIMER0->IF;
	  LETIMER0->IFC = intflags0;

	  NVIC_EnableIRQ(TIMER0_IRQn);

	  /*Clearing interrupt flags for TIMER0*/
	  intflags1 = LETIMER0->IF;
	  LETIMER0->IFC = intflags1;

	  blockSleep(EM1);

	  NVIC_EnableIRQ(TIMER1_IRQn);
	  unblockSleep(EM1);
}


/*GPIO setup routine*/
void GPIO_setup(){
	GPIO_DriveModeSet(LEDPORT, gpioDriveModeLowest);
	GPIO_DriveModeSet(LEDPORT_TEMPERATURE, gpioDriveModeLowest);

	GPIO_PinModeSet(LEDPORT,LEDPIN,gpioModePushPullDrive,0);
	GPIO_PinModeSet(LEDPORT_TEMPERATURE,LEDPIN_TEMPERATURE,gpioModePushPullDrive,0);

	GPIO_PinModeSet(I2C1_INT_PORT, I2C1_INT_PIN, gpioModeInput,0);	//setting up interrupt
	GPIO_PinModeSet(I2C1_VDD_PORT, I2C1_VDD_PIN, gpioModePushPull,0);	//setting up vdd

	GPIO_PinModeSet(I2C0_VDD_PORT, I2C0_VDD_PIN, gpioModePushPull,0);	//setting up vdd
}


/**************************************************************************//**
 * @brief Initializes the capacative touch system with LESENSE.
 *
 * @param[in] sensitivity
 *   An array of floats, indication of threshold level, in percent,
 *   of nominal count value.
 *
 *****************************************************************************/
void LETOUCH_Init(float sensitivity[]){

  uint8_t i;
  channels_used_mask = 0;
  num_channels_used = 0;

  /* Initialize channels used mask and threshold array for each channel */
  /* Uses the sensitivity array to deduce which channels to enable and how */
  /* many channels that are enabled */

  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    /* Init min and max values for each channel */
    channel_max_value[i] = 0;
    channel_min_value[i] = 0xffff;

    /* Add to channels used mask if sensitivity is not zero */
    if(sensitivity[i] != 0.0){
      channel_threshold_percent[i] = sensitivity[i];
      channels_used_mask |= (1 << i);
      num_channels_used++;
    }
  }

  /* Disable interrupts while initializing */
  INT_Disable();

  /* Setup GPIO. */
  LETOUCH_setupGPIO();
  /* Setup ACMP. */
  LETOUCH_setupACMP();
  /* Setup LESENSE. */
  LETOUCH_setupLESENSE();
  /* Do initial calibration "N_calibration_values * 10" times to make sure */
  /* it settles on values after potential startup transients */
  for(i = 0; i < NUMBER_OF_CALIBRATION_VALUES * 10; i++){
    LESENSE_ScanStart();
    LETOUCH_Calibration();
  }
  /* Setup RTC for calibration interrupt */
  LETOUCH_setupRTC();
  /* Initialization done, enable interrupts globally. */
  INT_Enable();

}

/***************************************************************************//**
 * @brief
 *   Get the buttons pressed variable, one bit for each channel pressed
 *   or'ed together.
 *
 * @return
 *   The touch buttons/pads that are in touched state is or'ed together and
 *   returned.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelsTouched(void){

  return buttons_pressed;
}

uint8_t channel_no_button(void){
	return channel_no;
}

/***************************************************************************//**
 * @brief
 *   Get the maximum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get maximum value for
 *
 * @return
 *   The maximum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMaxValue(uint8_t channel){
  return channel_max_value[channel];
}


/***************************************************************************//**
 * @brief
 *   Get the minimum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get minimum value for
 *
 * @return
 *   The minimum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMinValue(uint8_t channel){
  return channel_min_value[channel];
}


/**************************************************************************//**
 * @brief  Sets up the ACMP
 *****************************************************************************/
static void LETOUCH_setupACMP( void )
{
  /* Configuration structure for ACMP */
  /* See application note document for description of the different settings. */
  static const ACMP_CapsenseInit_TypeDef acmpInit =
  {
    .fullBias                 = true,            //Configured according to application note
    .halfBias                 = true,            //Configured according to application note
    .biasProg                 = 0x5,             //Configured according to application note
    .warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warmup time
    .hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
    .resistor                 = acmpResistor0,   //Configured according to application note
    .lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with captouch
    .vddLevel                 = 0x30,            //Configured according to application note
    .enable                   = false            //LESENSE enables the ACMP
  };

  /* Initialize ACMP in capsense mode*/
  ACMP_CapsenseInit(ACMP0, &acmpInit);
  ACMP_CapsenseInit(ACMP1, &acmpInit);
}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
static void LETOUCH_setupLESENSE( void )
{
  uint8_t i;
  /* LESENSE configuration structure */
  static const LESENSE_Init_TypeDef initLesense =
  {
    .coreCtrl         =
    {
      .scanStart    = lesenseScanStartPeriodic,
      .prsSel       = lesensePRSCh0,
      .scanConfSel  = lesenseScanConfDirMap,
      .invACMP0     = false,
      .invACMP1     = false,
      .dualSample   = false,
      .storeScanRes = false,
      .bufOverWr    = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
      .biasMode     = lesenseBiasModeDutyCycle,
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0x0
    },

    .perCtrl          =
    {
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode  = lesenseDACOutModeDisable,
      .dacPresc       = 0,
      .dacRef         = lesenseDACRefBandGap,
      .acmp0Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
      .acmp1Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl          =
    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = false,
      .hystPRS0  = false,
      .hystPRS1  = false,
      .hystPRS2  = false,
      .hystIRQ   = false,
      .prsCount  = false,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

  /* Channel configuration */
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,
    .enaPin        = true,
    .enaInt        = true,
    .chPinExMode   = lesenseChPinExDis,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkLF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x0,
    .sampleDelay   = SAMPLE_DELAY,
    .measDelay     = 0x0,
    .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
    .sampleMode    = lesenseSampleModeCounter,
    .intMode       = lesenseSetIntLevel,
    .cntThres      = 0x0,                   // Configured later by calibration function
    .compMode      = lesenseCompModeLess
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure channels */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      LESENSE_ChannelConfig(&initLesenseCh, i);
    }
  }

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LESENSE_SCAN_FREQUENCY);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
static void LETOUCH_setupGPIO( void )
{
  unsigned int i;
  /* Set GPIO pin mode to disabled for all active pins */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      GPIO_PinModeSet(LESENSE_CH_PORT, i, gpioModeDisabled, 0);
    }
  }
}

/**************************************************************************//**
 * @brief  Sets up the RTC
 *****************************************************************************/
void LETOUCH_setupRTC( void )
{
  /* RTC configuration */
  static const RTC_Init_TypeDef rtcInit =
  {
    .enable   = true,
    .debugRun = false,
    .comp0Top = true
  };

  RTC_Init(&rtcInit);

  /* Set the RTC calibration interrupt compare value */
  /* calibration interval defined in lesense_letouch_config.h */
  RTC_CompareSet( 0, CALIBRATION_INTERVAL * RTC_FREQ );

  RTC_IntEnable(RTC_IFS_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * Calibration function
*****************************************************************************/
void LETOUCH_Calibration( void ){
  int i,k;
  uint16_t nominal_count;
  static uint8_t calibration_value_index = 0;

  /* Wait for current scan to finish */
  while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

  /* Get position for first channel data in count buffer from lesense write pointer */
  k = ((LESENSE->PTR & _LESENSE_PTR_WR_MASK) >> _LESENSE_PTR_WR_SHIFT);

  /* Handle circular buffer wraparound */
  if(k >= num_channels_used){
    k = k - num_channels_used;
  }
  else{
    k = k - num_channels_used + NUM_LESENSE_CHANNELS;
  }

  /* Fill calibration values array with buffer values */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      calibration_value[i][calibration_value_index] = LESENSE_ScanResultDataBufferGet(k++);
    }
  }

  /* Wrap around calibration_values_index */
  calibration_value_index++;
  if(calibration_value_index >= NUMBER_OF_CALIBRATION_VALUES){
    calibration_value_index = 0;
  }

  /* Calculate max/min-value for each channel and set threshold */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      channel_max_value[i] = GetMaxValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);
      channel_min_value[i] = GetMinValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);

      nominal_count = channel_max_value[i];
      LESENSE_ChannelThresSet(i, 0x0,(uint16_t) (nominal_count - ((nominal_count * channel_threshold_percent[i])/100.0)) );
    }
  }
}



/**************************************************************************//**
 * Returns maximum value in input array of size N
*****************************************************************************/
static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t max = 0;

  for(i=0; i<N; i++)
  {
    if(max < A[i]){
      max = A[i];
    }
  }
  return max;
}


/**************************************************************************//**
 * Returns minimum value in input array of size N
*****************************************************************************/
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t min = 0xffff;

  for(i=0; i<N; i++)
  {
    if(A[i] < min){
      min = A[i];
    }
  }
  return min;
}


/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC, used for the calibration function
 *****************************************************************************/
void RTC_IRQHandler( void )
{
  /* Clear interrupt flag */
  RTC_IntClear(RTC_IFS_COMP0);

  LETOUCH_Calibration();

  /* Reset counter */
  RTC_CounterReset();
}


/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler( void )
{
  uint8_t channel, i, valid_touch;
  uint32_t interrupt_flags, tmp, channels_enabled;
  uint16_t threshold_value;

  /* Get interrupt flag */
  interrupt_flags = LESENSE_IntGet();
  /* Clear interrupt flag */
  LESENSE_IntClear(interrupt_flags);

  if(interrupt_flags & 0x0100)
	  channel_no = 8;
  else if(interrupt_flags &0x0800)
	  channel_no = 11;

  /* Interrupt handles only one channel at a time */
  /* therefore only first active channel found is handled by the interrupt. */
  for(channel = 0; channel < NUM_LESENSE_CHANNELS; channel++){
    if( (interrupt_flags >> channel) & 0x1 ){
      break;
    }
  }

  /* To filter out possible false touches, the suspected channel is measured several times */
  /* All samples should be below threshold to trigger an actual touch. */

  /* Disable other channels. */
  channels_enabled = LESENSE->CHEN;
  LESENSE->CHEN = 1 << channel;

   /* Evaluate VALIDATE_CNT results for touched channel. */
  valid_touch = 1;
  for(i = 0;i<VALIDATE_CNT;i++){
    /* Start new scan and wait while active. */
    LESENSE_ScanStart();
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

    tmp = LESENSE->SCANRES;
    if((tmp & (1 << channel)) == 0){
      valid_touch = 0;
    }
  }

  /* Enable all channels again. */
  LESENSE->CHEN = channels_enabled;

  if(valid_touch){
    /* If logic was switched clear button flag and set logic back, else set button flag and invert logic. */
    if(LESENSE->CH[channel].EVAL & LESENSE_CH_EVAL_COMP){
      buttons_pressed &= ~(1 << channel);
      LESENSE->CH[channel].EVAL &= ~LESENSE_CH_EVAL_COMP;

      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value -= 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
    }
    else{
      buttons_pressed |= (1 << channel);
      channel_no = 0;
      LESENSE->CH[channel].EVAL |= LESENSE_CH_EVAL_COMP;

      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value += 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
    }
  }

  /* Need to reset RTC counter so we don't get new calibration event right after buttons are pushed/released. */
  RTC_CounterReset();

}


/***************************************************************************
 * setting up i2c for the temp, pressure and humidity sensor
 **************************************************************************/
// This function sets up the I2C0 peripheral by configuring the necessary GPIO pins
void setupI2C0(void)
{
  // Using default settings
  I2C_Init_TypeDef i2c_Init = I2C_INIT_DEFAULT;

  /* Using PD6 (SDA) and PD7 (SCL) */
  GPIO_PinModeSet(I2C0_SDA_PORT, I2C0_SDA_PIN, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeWiredAnd, 1);


 for (int i = 0; i < 9; i++)
    {
      GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeWiredAnd, 0);
      GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeWiredAnd, 1);
    }

  /* Enable pins at location 1 */
  I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (1 << _I2C_ROUTE_LOCATION_SHIFT);

  /* Initializing the I2C */
  I2C_Init(I2C0, &i2c_Init);


  if (I2C0->STATE & I2C_STATE_BUSY) //check if busy, if busy then abort
  {
      I2C0->CMD = I2C_CMD_ABORT;
  }
}


/**********************************************************
//This I2C function writes values onto the i2c slave
**********************************************************/
void I2C0_Write(uint8_t addr, uint8_t data){

I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);  //sending slave address
I2C0->CMD = I2C_CMD_START;
I2C0->IFC = I2C_IFC_START;  				 //check for start flag in IF reg and then clear it

while((I2C0->IF & I2C_IF_ACK) == 0);
I2C0->IFC = I2C_IFC_ACK;

I2C0 -> TXDATA = addr;                    // loading command register value
while((I2C0->IF & I2C_IF_ACK) == 0);
I2C0->IFC = I2C_IFC_ACK;

I2C0 -> TXDATA = data;					  //loading data value
while((I2C0->IF & I2C_IF_ACK) == 0);
I2C0->IFC = I2C_IFC_ACK;

I2C0 -> CMD = I2C_CMD_STOP;
while ((I2C0->IF & I2C_IF_MSTOP) ==  0);  //check for Mstop
I2C0->IFC=I2C_IFC_MSTOP;

}


uint16_t I2C0_Read(uint8_t addr){
	  // addr is the command register address
		uint8_t reg_address = addr;

		I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);	//0 is for write
		I2C0 -> CMD = I2C_CMD_START;
		I2C0 -> IFC = I2C_IFC_START;

		while((I2C0->IF & I2C_IF_ACK) == 0);
		I2C0 -> IFC = I2C_IFC_ACK; //Clear ACK flag in IFC

		I2C0 -> TXDATA = reg_address;                    // loading command register value
		while((I2C0->IF & I2C_IF_ACK) == 0);
		I2C0->IFC = I2C_IFC_ACK;

		I2C0 -> CMD = I2C_CMD_START;			// sr bit
		I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 1);	//1 is for read

		while((I2C0->IF & I2C_IF_ACK) == 0);
		I2C0->IFC = I2C_IFC_ACK;

		while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
	  data0 =  I2C0->RXDATA;
	  I2C0->CMD =I2C_CMD_NACK;		//send NACK to inform the slave that we don't need more data

	  I2C0->CMD = I2C_CMD_STOP;
	  while ((I2C0->IF & I2C_IF_MSTOP) ==  0);
	  I2C0->IFC=I2C_IFC_MSTOP;          // check for MSTOP and then clear it

		return data0;
}


void I2C0_sensor(void){

		  // 0x88 is the read calibration register address
			uint8_t reg_address = 0x88;

			I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);	//0 is for write
			I2C0 -> CMD = I2C_CMD_START;
			I2C0 -> IFC = I2C_IFC_START;

			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0 -> IFC = I2C_IFC_ACK; //Clear ACK flag in IFC

			I2C0 -> TXDATA = reg_address;                    // loading command register value
			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0->IFC = I2C_IFC_ACK;

			I2C0 -> CMD = I2C_CMD_START;			// sr bit
			I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 1);	//1 is for read

			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0->IFC = I2C_IFC_ACK;

			while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    data0 =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_NACK;		//send NACK to inform the slave that we don't need more data

		    I2C0->CMD = I2C_CMD_STOP;
		  	while ((I2C0->IF & I2C_IF_MSTOP) ==  0);
		  	I2C0->IFC=I2C_IFC_MSTOP;          // check for MSTOP and then clear it

	}



/***************************************************************************
 * Writing the driver function to read data from the slave
 **************************************************************************/
uint16_t I2C1_Read(uint8_t addr){

	    // addr is the command register address
		uint8_t address = addr;

		I2C1 -> TXDATA = (SLAVE_ADDRESS << 1 | 0);	//0 is for write
		I2C1 -> CMD = I2C_CMD_START;
		I2C1 -> IFC = I2C_IFC_START;

		while((I2C1->IF & I2C_IF_ACK) == 0);
		I2C1 -> IFC = I2C_IFC_ACK; //Clear ACK flag in IFC

		I2C1 -> TXDATA = address;                    // loading command register value
		while((I2C1->IF & I2C_IF_ACK) == 0);
		I2C1->IFC = I2C_IFC_ACK;


		I2C1 -> CMD = I2C_CMD_START;			// sr bit
		I2C1 -> TXDATA = (SLAVE_ADDRESS << 1 | 1);	//1 is for read

		while((I2C1->IF & I2C_IF_ACK) == 0);
		I2C1->IFC = I2C_IFC_ACK;

		while ((I2C1->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
	  data0 =  I2C1->RXDATA;
		I2C1->CMD =I2C_CMD_ACK;

	  while ((I2C1->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
	  data1 =  I2C1->RXDATA;
	  I2C1->CMD =I2C_CMD_NACK;		//send NACK to inform the slave that we don't need more data

	  I2C1->CMD = I2C_CMD_STOP;
	  while ((I2C1->IF & I2C_IF_MSTOP) ==  0);
	  I2C1->IFC=I2C_IFC_MSTOP;      // check for MSTOP and then clear it

		return data1*256 + data0;			//returning the ADC value
}


//Calibration routine for ULFRCO to get a more accurate value.
void Calibration(void){
	float count_ratio = 1;
	CMU_OscillatorEnable(cmuOsc_LFXO,true,true);
	CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);

	LETIMER_Init_TypeDef letimerInit =
		  {
		  .enable         = false,                   /* Start counting when init completed. */
		  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
		  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
		  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
		  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
		  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
		  .out0Pol        = 0,                      /* Idle value for output 0. */
		  .out1Pol        = 0,                      /* Idle value for output 1. */
		  .ufoa0          = letimerUFOANone,
		  .ufoa1          = letimerUFOANone,
		  .repMode        = letimerRepeatOneshot      /* Count only for one cycle */
		  };

	/*Initializing the LETIMER*/
	LETIMER_Init(LETIMER0, &letimerInit);

	LETIMER0 -> CNT = LFXO_COUNTS;			/*Loading the count value of LETIMER with LFXO counter value*/
	TIMER0 -> CNT =0;
	TIMER1 -> CNT =0;

	TIMER_Setup();

	LETIMER_Enable(LETIMER0,true);
	TIMER_Enable(TIMER0,true);						/*Enabling the TIMER0*/
	TIMER_Enable(TIMER1,true);						/*Enabling the TIMER1*/

	while((LETIMER0->CNT)!=0);						/*Waiting till LETIMER counter comes to 0*/
	lfxo_count = (((TIMER1->CNT) << 16)|(TIMER0->CNT));

	//Disabling timers
	LETIMER_Enable(LETIMER0,false);
	TIMER_Enable(TIMER0,false);
	TIMER_Enable(TIMER1,false);

	CMU_OscillatorEnable(cmuOsc_LFXO,false,false);
	CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);
	CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);

	LETIMER0 -> CNT = ULFRCO_COUNTS;
	TIMER0 -> CNT = 0;
	TIMER1 -> CNT = 0;

	LETIMER_Init(LETIMER0, &letimerInit);
	TIMER_Setup();
	LETIMER_Enable(LETIMER0,true);

	TIMER_Enable(TIMER0,true);
	TIMER_Enable(TIMER1,true);

	while((LETIMER0->CNT)!=0);
	ulfrco_count= ((TIMER1->CNT) << 16)|(TIMER0->CNT);
	count_ratio = (float)lfxo_count /(float) ulfrco_count;

	ULFRCO_COUNTS= ULFRCO_COUNTS * count_ratio;		/*Multiplying the ratio to the count value to calibrate ULFRCO*/
	TIMER_Enable(TIMER0,false);						/*Disabling the TIMER0*/
	TIMER_Enable(TIMER1,false);

	LETIMER_Enable(LETIMER0,false);					/*Disabling the LETIMER*/

}


/* The below function convertToCelcius uses the algorithm and IP from Silicon Labs. It has directly
 been taken from their data sheet and the below code logic should be credited to them. */
// This function converts the sample value into temperature using the temperature gradient
// value given to us by Silicon Labs

float convertToCelsius(float sample){
	float temp;
	float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >>
			_DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
			>> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

	float tgrad = -6.27;
	temp = (cal_temp_0 - ((cal_value_0 - sample)/tgrad));

	return temp;
}


// This function sets up the I2C1 peripheral by configuring the necessary GPIO pins
void setupI2C1(void)
{
  // Using default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

  /* Using PC4 (SDA) and PC5 (SCL) */
  GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAnd, 1);


 for (int i = 0; i < 9; i++)
    {
      GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 0);
      GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 1);
    }

  /* Enable pins at location 1 */
  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (0 << _I2C_ROUTE_LOCATION_SHIFT);

  /* Initializing the I2C */
  I2C_Init(I2C1, &i2cInit);


  if (I2C1->STATE & I2C_STATE_BUSY) //check if busy, if busy then abort
  {
      I2C1->CMD = I2C_CMD_ABORT;
  }
}


//load power management routine for power on
void power_up(){
	GPIO_PinOutSet(I2C1_VDD_PORT, I2C1_VDD_PIN);
	stabilisation();    // stabilization routine
	setupI2C1();	// setting up the i2c device
	I2C1_sensor();	// transmitting the threshold, persistence and gain parameters
}


// load power management routine for shut-down
void power_down(){
	uint8_t address = 0xC6; // setting clear bit in command register to 1.
	uint8_t data = 0x04;	// disabling interrupt
	I2C1_Write(address, data);
	GPIO_PinOutClear(I2C1_VDD_PORT, I2C1_VDD_PIN);	//clearing the PD0
}


/**********************************************************
//This I2C function writeS values onto the i2c slave
**********************************************************/
void I2C1_Write(uint8_t addr, uint8_t data){
	// addr is the command register address

	I2C1 -> TXDATA = (SLAVE_ADDRESS << 1 | 0);  //sending slave address
	I2C1->CMD = I2C_CMD_START;
	I2C1->IFC=I2C_IFC_START;//check for start flag in IF reg and then clear it

	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;

	I2C1 -> TXDATA = addr;                    // loading command register value
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;

	I2C1 -> TXDATA = data;					//loading data value
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;

	I2C1 -> CMD = I2C_CMD_STOP;
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0);  //check for Mstop
	I2C1->IFC=I2C_IFC_MSTOP;

}

/****************************************************************************************
// Initializing the I2C sensor by setting the persistence, low gain, threshold parameters
 * **************************************************************************************
 */
void I2C1_sensor(void){
	INT_Disable();

	//defining persistence to 4
	uint8_t address = 0x86; // interrupt register is being accessed
	uint8_t data = ( INTR <<4)|(PERSISTENCE) ;
	I2C1_Write(address, data);

	//defining threshold low low register
	address = 0x82; // threshold low low register is being accessed
	data = 0x0f;
	I2C1_Write(address, data);

	//defining threshold low high register
	address = 0x83; // threshold low low register is being accessed
	data = 0x00;
	I2C1_Write(address, data);

	//defining threshold high low register
	address = 0x84; // threshold high low register is being accessed
	data = 0x00;
	I2C1_Write(address, data);

	//defining threshold high high register
	address = 0x85; // threshold high high register is being accessed
	data = 0x08;
	I2C1_Write(address, data);

	//defining low gain and integration time
	address = 0x81; // timing register being accessed
	data = 0x01;   // only INTEG=0b01, rest all are 0
	I2C1_Write(address, data);

	I2C1_Write(COMMAND,0x03);     //writing value on to command register to power up the sensor

	for(int i =0; i<10000; i++){        //setting a delay
	}

	INT_Enable();
}

//setup BME
bool setup_BME(uint8_t operationMode){
	setupI2C0();
	if(!BME_verify_ChipId()){
		return 0;
	}
	BME_OperationMode = operationMode;
	BME_writeConfig();
	BME_readCalibrationRegisters();
	return 1;
}

//configures the BME registers
void BME_writeConfig(void){
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | BME_OperationMode;
    uint8_t ctrl_hum_reg  = osrs_h;
    uint8_t config_reg    = (time_standby << 5) | (iir_filter << 2) | spi_enable;
    I2C0_Write(BME_CTRL_MEAS, ctrl_meas_reg);
    I2C0_Write(BME_CTRL_HUM, ctrl_hum_reg);
    I2C0_Write(BME_CTRL_MEAS, ctrl_meas_reg);
    I2C0_Write(BME_CONFIG, config_reg);
}

//verifying chip id
bool BME_verify_ChipId(void){
	uint8_t chip_id = I2C0_Read(BME_CHIP_ID_REGISTER);
	if (chip_id != BME_CHIP_ID_DEFAULT ) {
	    return 0;
	}
	return 1;
}


//reads the calibration registers
void BME_readCalibrationRegisters(void){

			uint8_t reg_address = 0x88;

			I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);	//0 is for write
			I2C0 -> CMD = I2C_CMD_START;
			I2C0 -> IFC = I2C_IFC_START;

			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0 -> IFC = I2C_IFC_ACK; //Clear ACK flag in IFC

			I2C0 -> TXDATA = reg_address;                    // loading command register value
			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0->IFC = I2C_IFC_ACK;

			I2C0 -> CMD = I2C_CMD_START;			// sr bit
			I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 1);	//1 is for read

			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0->IFC = I2C_IFC_ACK;

			while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		  	while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		  	msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_T1 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_T2 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_T3 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P1 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P2 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P3 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P4 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK
		    calib_dig_P5 = (msb << 8) | lsb;
		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P6 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P7 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_P8 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		  	lsb =  I2C0->RXDATA;
		  	I2C0->CMD =I2C_CMD_ACK;

		  	while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		  	msb =  I2C0->RXDATA;
		  	I2C0->CMD =I2C_CMD_NACK;		//send NACK

		  	calib_dig_P9 = (msb << 8) | lsb;

		    I2C0->CMD = I2C_CMD_STOP;
		  	while ((I2C0->IF & I2C_IF_MSTOP) ==  0);
		  	I2C0->IFC=I2C_IFC_MSTOP;          // check for MSTOP and then clear it

		  	reg_address = 0xA1;
		  	I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);	//0 is for write
		  	I2C0 -> CMD = I2C_CMD_START;
		  	I2C0 -> IFC = I2C_IFC_START;

		  	while((I2C0->IF & I2C_IF_ACK) == 0);
		  	I2C0 -> IFC = I2C_IFC_ACK; //Clear ACK flag in IFC

		  	I2C0 -> TXDATA = reg_address;                    // loading command register value
		  	while((I2C0->IF & I2C_IF_ACK) == 0);
		  	I2C0->IFC = I2C_IFC_ACK;

		  	I2C0 -> CMD = I2C_CMD_START;			// sr bit
		  	I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 1);	//1 is for read

		  	while((I2C0->IF & I2C_IF_ACK) == 0);
		  	I2C0->IFC = I2C_IFC_ACK;

		  	while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		  	msb =  I2C0->RXDATA;
		  	I2C0->CMD =I2C_CMD_NACK;

		  	calib_dig_H1 = msb;

		    I2C0->CMD = I2C_CMD_STOP;
		  	while ((I2C0->IF & I2C_IF_MSTOP) ==  0);
		  	I2C0->IFC=I2C_IFC_MSTOP;          // check for MSTOP and then clear it

		  	reg_address = 0xE1;

		  	I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);	//0 is for write
		  	I2C0 -> CMD = I2C_CMD_START;
		  	I2C0 -> IFC = I2C_IFC_START;

		  	while((I2C0->IF & I2C_IF_ACK) == 0);
		  	I2C0 -> IFC = I2C_IFC_ACK; 	//Clear ACK flag in IFC

		  	I2C0 -> TXDATA = reg_address;                    // loading command register value
		  	while((I2C0->IF & I2C_IF_ACK) == 0);
		  	I2C0->IFC = I2C_IFC_ACK;

		  	I2C0 -> CMD = I2C_CMD_START;			// sr bit
		  	I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 1);	//1 is for read

		  	while((I2C0->IF & I2C_IF_ACK) == 0);
		  	I2C0->IFC = I2C_IFC_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_H2 = (msb << 8) | lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_H3 = lsb;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    msb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_H4 = (lsb << 4) | (0x0f & msb);

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_ACK;		//send ACK

		    calib_dig_H5 = (lsb << 4) | ((msb >> 4) & 0x0f);

		    while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);//check for flag to check data in recieve buffer, if set then get data from recieve buffer
		    lsb =  I2C0->RXDATA;
		    I2C0->CMD =I2C_CMD_NACK;		//send NACK

		    calib_dig_H6 = lsb;

		    I2C0->CMD = I2C_CMD_STOP;
		  	while ((I2C0->IF & I2C_IF_MSTOP) ==  0);
		  	I2C0->IFC=I2C_IFC_MSTOP;          // check for MSTOP and then clear it
}


//reads the raw sensor data
void BME_readSensorData(void){
	uint8_t msb, lsb, xlsb;
	//F7 - pressure
	uint8_t reg_address = 0xF7;
	I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 0);	//0 is for write
	I2C0 -> CMD = I2C_CMD_START;
	I2C0 -> IFC = I2C_IFC_START;
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0 -> IFC = I2C_IFC_ACK;                //Clear ACK flag in IFC
	I2C0 -> TXDATA = reg_address;             // loading command register value
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;
	I2C0 -> CMD = I2C_CMD_START;			        // sr bit
	I2C0 -> TXDATA = (BME_SLAVE_ADDR << 1 | 1);	//1 is for read

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;

	while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  msb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  lsb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  xlsb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

	//raw pressure data
  raw_pres = (msb << 12) | (lsb << 4) | (xlsb >> 4);

  //FA - Temperature
  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  msb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  lsb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  xlsb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

  //raw temperature data
  raw_temp = (msb << 12) | (lsb << 4) | (xlsb >> 4);

  //FD - Humidity
  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  msb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_ACK;

  while ((I2C0->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
  lsb =  I2C0->RXDATA;
  I2C0->CMD =I2C_CMD_NACK;

  //raw humidity data
  raw_hum = (msb << 8) | lsb;

  //stopping the module
  I2C0->CMD = I2C_CMD_STOP;
	while ((I2C0->IF & I2C_IF_MSTOP) ==  0);
	I2C0->IFC=I2C_IFC_MSTOP;          // check for MSTOP and then clear it

}

//getting the raw values
unsigned long int BME_getTemperatureRaw(void){
	return raw_temp;
}

unsigned long int BME_getPressureRaw(void){
	return raw_pres;
}

unsigned long int BME_getHumidityRaw(void){
	return raw_hum;
}


//correcting the raw values by applying calibration by using Silicon Labs' API's
signed long int BME_calibration_Temp(signed long int adc_T){
    signed long int a, b, Temp;
    a = ((((adc_T >> 3) - ((signed long int)calib_dig_T1<<1))) * ((signed long int)calib_dig_T2)) >> 11;
    b = (((((adc_T >> 4) - ((signed long int)calib_dig_T1)) * ((adc_T>>4) - ((signed long int)calib_dig_T1))) >> 12) * ((signed long int)calib_dig_T3)) >> 14;
    t_fine = a + b;
    Temp = (t_fine * 5 + 128) >> 8;
    return Temp;
}

unsigned long int BME_calibration_Press(signed long int adc_P){
    signed long int a, b;
    unsigned long int Pressure;
    a = (((signed long int)t_fine)>>1) - (signed long int)64000;
    b = (((a>>2) * (a>>2)) >> 11) * ((signed long int)calib_dig_P6);
    b = b + ((a*((signed long int)calib_dig_P5))<<1);
    b = (b>>2)+(((signed long int)calib_dig_P4)<<16);
    a = (((calib_dig_P3 * (((a>>2)*(a>>2)) >> 13)) >>3) + ((((signed long int)calib_dig_P2) * a)>>1))>>18;
    a = ((((32768+a))*((signed long int)calib_dig_P1))>>15);
    if (a == 0){
        return 0;
    }
    Pressure = (((unsigned long int)(((signed long int)1048576)-adc_P)-(b>>12)))*3125;
    if(Pressure<0x80000000){
    	Pressure = (Pressure << 1) / ((unsigned long int) a);
    }else{
    	Pressure = (Pressure / (unsigned long int)a) * 2;
    }
    a = (((signed long int)calib_dig_P9) * ((signed long int)(((Pressure>>3) * (Pressure>>3))>>13)))>>12;
    b = (((signed long int)(Pressure>>2)) * ((signed long int)calib_dig_P8))>>13;
    Pressure = (unsigned long int)((signed long int)Pressure + ((a + b + calib_dig_P7) >> 4));
    return Pressure;
}

unsigned long int BME_calibration_Hum(signed long int adc_H){
	  signed long int a;

	    a = (t_fine - ((signed long int)76800));
	    a = (((((adc_H << 14) -(((signed long int)calib_dig_H4) << 20) - (((signed long int)calib_dig_H5) * a)) +
	              ((signed long int)16384)) >> 15) * (((((((a * ((signed long int)calib_dig_H6)) >> 10) *
	              (((a * ((signed long int)calib_dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
	              ((signed long int) calib_dig_H2) + 8192) >> 14));
	   a = (a - (((((a >> 15) * (a >> 15)) >> 7) * ((signed long int)calib_dig_H1)) >> 4));
	   a = (a < 0 ? 0 : a);
	   a = (a > 419430400 ? 419430400 : a);
	   return (unsigned long int)(a >> 12);
}


//actual sensor values after correcting it
void BME_readActualSensorData(void){
	//read raw sensor values
	BME_readSensorData();

	temp_act = BME_calibration_Temp(raw_temp);
	press_act = BME_calibration_Press(raw_pres);
	hum_act = BME_calibration_Hum(raw_hum);

}

//getting actual sensor values after calibration
signed long int BME_getTemperature(void){
	return temp_act;
}

unsigned long int BME_getPressure(void){
	return press_act;
}

unsigned long int BME_getHumidity(void){
	return hum_act;
}


// Routine for setting up ACMP
void ACMP_setup(void)
{

  ACMP_Init_TypeDef acmp_init =
  {
    false,                              /* Full bias current*/
    true,                               /* Half bias current */
    5,                                  /* Biasprog current configuration */
    false,                              /* Enable interrupt for falling edge */
    true,                               /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel7,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    true,                               /* Enable low power mode */
    REFERENCE_LEVEL,                    /* Vdd reference scaling */
    true,                               /* Enable ACMP */
  };


  ACMP_Init(ACMP0, &acmp_init);
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);
  ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);   /* Enable edge interrupt */

  while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;

  blockSleep(energyMode);

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(ACMP0_IRQn);
  NVIC_EnableIRQ(ACMP0_IRQn);

}


/**************************************************************************
 * Interrupt Service Routine Odd GPIO Interrupt Line
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  INT_Disable();

  uint32_t flags = GPIO->IF;
  // clear flag for PD1 interrupt
  GPIO->IFC = flags;
  adc_sensor_count = I2C1_Read(0xAC);   	// Reading from data0low & data0high
  GPIO->IFC = GPIO->IF;
  
  INT_Enable();
}


// The ACMP0 interrupt handler routine
void ACMP0_IRQHandler(void)
{
	INT_Disable();
	ACMP0->IFC = ACMP0->IF;
	INT_Enable();
}


// setting a stabilization routine for I2C
void stabilisation(void){
	for(int i =0; i<10000; i++){}
}


// Setting up the LETIMER0
void LETIMER_Setup(){
	int Comp0_Init;
	int Comp1_Init;

	int prescaler;
	int intflags;

	const LETIMER_Init_TypeDef letimerInit =
	  {
	  .enable         = false,                   /* Start counting when init completed. */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
	  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
	  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	  .out0Pol        = 0,                      /* Idle value for output 0. */
	  .out1Pol        = 0,                      /* Idle value for output 1. */
	  .ufoa0          = letimerUFOANone,
	  .ufoa1          = letimerUFOANone,
	  .repMode        = letimerRepeatFree       /* Count until stopped */
	  };

	  /* Initialize LETIMER */
	  LETIMER_Init(LETIMER0, &letimerInit);

	  if(energyMode==EM3)
		  Comp0_Init = OFF_TIME * ULFRCO_COUNTS;

	  else
	  {
		  Comp0_Init = OFF_TIME * LFXO_COUNTS;
		  /*Now we will use a prescaler*/
		  prescaler = OFF_TIME / 2;
	  	  CMU->LFAPRESC0 &=  0xfffff0ff;
	  	  CMU->LFAPRESC0 |= prescaler << 8;
	  	  prescaler = 1 << prescaler;
		  Comp0_Init = OFF_TIME * (LFXO_COUNTS / prescaler);
	  }

	  LETIMER0 -> CNT = Comp0_Init;
	  LETIMER_CompareSet(LETIMER0,0,Comp0_Init);

	  if(energyMode==EM3)
		  Comp1_Init = ON_TIME * ULFRCO_COUNTS;
	  else
		  Comp1_Init = ON_TIME * (LFXO_COUNTS / prescaler);

	  LETIMER_CompareSet(LETIMER0,1,Comp1_Init);

	  while(LETIMER0->SYNCBUSY!=0);							/*Waiting for LETIMER0 synch bit to be cleared*/

	  intflags=LETIMER0 -> IF;
	  LETIMER0 -> IFC = intflags;
	  LETIMER0->IEN=((LETIMER_IEN_COMP1)|(LETIMER_IEN_UF));

	  blockSleep(energyMode);

	  NVIC_EnableIRQ(LETIMER0_IRQn);
}


//setup function for LEUART
void initLeuart(void)
{
	int int_flags;

	if(energyMode == EM3){
		CMU_OscillatorEnable(cmuOsc_LFRCO,true,true);
		CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFRCO);		/*Selecting the LFRCO clock*/
	}
	else
	{
		CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFXO);		/*Selecting the LFXO clock*/
	 }

	CMU_ClockEnable(cmuClock_LEUART0,true);						/*Enabling clock for LEUART0*/
	int i;

	for(i=0;i<100;i++){}

	int_flags= LEUART0->IF;
	LEUART0->IFC = int_flags;

	/* Defining the LEUART1 initialization data */
	LEUART_Init_TypeDef leuart0Init =
	{
			.enable   = leuartEnableTx,       /* Activate data reception on LEUn_TX pin. */
			.refFreq  = 0,                    /* Inherit the clock frequency from the LEUART clock source */
			.baudrate = 9600,                 /* Baudrate = 9600 bps */
			.databits = leuartDatabits8,      /* Each LEUART frame contains 8 databits */
			.parity   = leuartNoParity,       /* No parity bits in use */
			.stopbits = leuartStopbits1,      /* Setting the number of stop bits in a frame to 1 bit periods */
	};

  /* Reseting and initializing LEUART0 */
  LEUART_Reset(LEUART0);
  LEUART_Init(LEUART0, &leuart0Init);

  /* Enable GPIO for LEUART0. TX is on D4 */
  GPIO_PinModeSet(gpioPortD,                /* GPIO port */
                  4,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */

	LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;
	LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
	LEUART0->CTRL |= LEUART_CTRL_AUTOTRI;

	while (LEUART0->SYNCBUSY) { }

  LEUART_IntEnable(LEUART0,LEUART_IEN_TXBL);			/*Enabling the TXBL interrupts*/
	NVIC_EnableIRQ(LEUART0_IRQn);

}


//LEUART IRQ Handler
void LEUART0_IRQHandler(void)
{
		INT_Disable();
		if(energyMode == EM3)
		{
			CMU_OscillatorEnable(cmuOsc_LFRCO,true,true);
		}

		uint8_t flags = LEUART0->IF;
		LEUART0->IFC = flags;
		LEUART0 -> IEN &= ~LEUART_IF_TXBL;

		/* Enable Transmission*/
		LEUART0->CMD |= LEUART_CMD_TXEN;
		LEUART0->CMD |= LEUART_CMD_RXEN;

		//for sending temperature
		//Sending integer part
		LEUART0->TXDATA = integer_average;
		while((LEUART0->IF & LEUART_IF_TXC)==0);
		flags = LEUART0->IF;
		LEUART0->IFC = flags;

		average_fractional = average * 10;
		average_fractional_2 = average_fractional%10;

		//sending fractional part
		LEUART0->TXDATA = average_fractional_2;
		while((LEUART0->IF & LEUART_IF_TXC)==0);
		flags = LEUART0->IF;
		LEUART0->IFC = flags;

		//transmitting pressure values
		//Sending integer part
		LEUART0->TXDATA = integer_average_press1;
		while((LEUART0->IF & LEUART_IF_TXC)==0);
		flags = LEUART0->IF;
		LEUART0->IFC = flags;

		//Sending integer part
		LEUART0->TXDATA = integer_average_press2;
		while((LEUART0->IF & LEUART_IF_TXC)==0);
		flags = LEUART0->IF;
		LEUART0->IFC = flags;

		average_fractional_press = actual_pressure * 10;
		average_fractional__press2 = average_fractional_press%10;

		//sending fractional part
		LEUART0->TXDATA = average_fractional__press2;
		while((LEUART0->IF & LEUART_IF_TXC)==0);
		flags = LEUART0->IF;
		LEUART0->IFC = flags;

		//transmitting sign value
		LEUART0 -> TXDATA = neg_value;
		while((LEUART0->IF & LEUART_IF_TXC)==0);
		flags = LEUART0->IF;
		LEUART0->IFC = flags;

		//LEUART0 -> IEN &= ~LEUART_IF_TXBL;
		LEUART0->CMD |= LEUART_CMD_TXDIS;
		INT_Enable();

}


/*Interrupt routine*/
void LETIMER0_IRQHandler(void)
{

/*Setting the dutycycle and period alternatively on LETIMER*/
 INT_Disable();
 int IntFlags;
 IntFlags=LETIMER0->IF;


 if((IntFlags & LETIMER_IF_UF)!=0)
 {
     LETIMER0->IFC=IntFlags;

     if(DMA_ON!=0)
     {
         DMA_ActivateBasic(DMA_CHANNEL_ADC,
	    	                       true,
	    	                       false,
	    	                       (void *)ramBufferAdcData,
	    	                       (void *)&(ADC0->SINGLEDATA),
	    	                       ADCSAMPLES - 1);
        transferActive=true;
     }

   /*Blocking the sleep mode to EM1 for ADC*/
     blockSleep(EM1);
   /*Starting ADC Conversion*/
     ADC_Start(ADC0, adcStartSingle);

     if(mod==1)
     {
     GPIO_PinOutSet(gpioPortD, 0);	//vdd for i2c1
     stabilisation();
     setupI2C1();
     I2C1_sensor();

     setup_BME(BME_MODE_FORCED);
     BME_readActualSensorData();
     actual_temperature = (BME_getTemperature()/100.00) + ((BME_getTemperature()%100)/100.00);
     actual_pressure = (BME_getPressure()/100.00) + ((BME_getPressure()%100)/100.00);
     actual_humidity = (BME_getHumidity()/1024.00) + ((BME_getHumidity()%1024)/1024.00);
     ftoa(actual_humidity,text,10000);
     SegmentLCD_Write(text);

     integer_average_press1 = (uint8_t)(actual_pressure/10);
     integer_average_press2 = ((uint16_t)actual_pressure) % 10;

     //applying hypsometric formula to find altitude
     altitude = (pow((1013.25/actual_pressure), (1/5.257))-1) * (actual_temperature + 273.15)/0.0065;


     //Weather trigger points for the two on-board leds
     //weather_code 0 : Pleasant weather
     //weather_code 1 : Fair weather
     //weather_code 2 : Uncomfortable weather
     //weather_code 3 : Bad weather

     //pleasant
     if((average>12 && average<25) && (actual_pressure>800 && actual_pressure<950) && (actual_humidity>35 && actual_humidity<62)){
    	 weather_code = 0;
    	 GPIO_PinOutClear(LEDPORT, LEDPIN);
    	 GPIO_PinOutClear(LEDPORT_TEMPERATURE, LEDPIN_TEMPERATURE);
     }

     //fair
     else if(((average>5 && average<15)||(average>25 && average<32)) && ((actual_pressure>700 && actual_pressure<800)||(actual_pressure>925 && actual_pressure<1010)) && ((actual_humidity>25 && actual_humidity<35)||(actual_humidity>63 && actual_humidity<70)))
    {
    	 weather_code = 1;
    	 GPIO_PinOutSet(LEDPORT, LEDPIN);
    	 GPIO_PinOutClear(LEDPORT_TEMPERATURE, LEDPIN_TEMPERATURE);
    }

     //uncomfortable
     else if(((average>-5 && average<5)||(average>32 && average<38)) && ((actual_pressure>650 && actual_pressure<700)||(actual_pressure>1010 && actual_pressure<1050)) && ((actual_humidity>15 && actual_humidity<25)||(actual_humidity>70 && actual_humidity<79)))
    {
    	 weather_code = 2;
    	 GPIO_PinOutClear(LEDPORT, LEDPIN);
    	 GPIO_PinOutSet(LEDPORT_TEMPERATURE, LEDPIN_TEMPERATURE);
    }

     //bad
     else if((average<-5||average>38) && (actual_pressure<650||actual_pressure>1050) && (actual_humidity<15||actual_humidity>80))
    {
    	 weather_code = 3;
    	 GPIO_PinOutSet(LEDPORT, LEDPIN);
    	 GPIO_PinOutSet(LEDPORT_TEMPERATURE, LEDPIN_TEMPERATURE);
    }


     /* Configure PE1 interrupt on falling edge */
     	  GPIO_ExtIntConfig(gpioPortD, 1,1, false, true, true);
     	  GPIO->IFC = GPIO->IF;

     	  /* Enable GPIO_ODD interrupt vector in NVIC */
     	  NVIC_EnableIRQ(GPIO_ODD_IRQn);
     }

     if(mod==2){
    	 if(weather_code == 0){
    		         SegmentLCD_Write("Good");
    	 }
    	 else if(weather_code == 1){
    	     	     SegmentLCD_Write("Fair");
    	     	 }
    	 if(weather_code == 2){
    	     		 SegmentLCD_Write("Unpleasant");
    	     	 }
    	 if(weather_code == 3){
    	     		 SegmentLCD_Write("Bad");
    	     	 }
    	 GPIO_ExtIntConfig(gpioPortD, 1,1, false, true, false);
    	 			  			GPIO_PinOutClear(gpioPortD, 0);
    	 						GPIO->IFC = GPIO->IF;
    	 			  			//mod=0;
    	 			  			unblockSleep(EM1);
    	 			  			blockSleep(energyMode);

     }

     else if(mod==3)
     {
    	 if(adc_sensor_count > 1500)     //condition of sunny
    	   {
    		 SegmentLCD_Write("Sunny");
    	   }
    	 else{
    		 SegmentLCD_Write("Dark");
    	 }
     }

     else if(mod==4){
    	 ftoa(altitude,text,10000);
    	 SegmentLCD_Write(text);
     }

     if(mod==4)
     {
    	mod=1;
     }

     else
     {
    	 mod++;
     }
  }

INT_Enable();
}


// Activates or disables LED
void LED_Activate(bool state){
	if(state)
		GPIO_PinOutSet(LEDPORT,LEDPIN);						/*Switching on the LED*/
	else
		GPIO_PinOutClear(LEDPORT,LEDPIN);					/*Switching off the LED*/
}



/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main(void)
{  
    /* Initialize chip */
	  CHIP_Init();
  
  	CMU_setup();												/*Calling the CMU setup routine*/
  	GPIO_setup();												/*Calling the GPIO setup routine*/

  	if(CALIBRATION == 1)
  	{Calibration();}

  	CMU_setup();
  	setupAdc();
  	ADC0->CMD |= ADC_CMD_SINGLESTOP;                            // Initially keep the ADC off

  	if(DMA_ON==1)                                               // Setup dma only if DMA_ON = 1
  		setupDma();

  	LETIMER_Setup();											/*Calling the LETIMER setup routine*/
  	LETIMER_Enable(LETIMER0,true);

  	/* Bitmask for the currently touched channels */
  	uint16_t channels_touched = 0;
    float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};

    /* Init Capacitive touch for channels configured in sensitivity array */
    LETOUCH_Init(sensitivity);

    /* If any channels are touched while starting, the calibration will not be correct. */
    /* Wait while channels are touched to be sure we continue in a calibrated state. */
    while(LETOUCH_GetChannelsTouched() != 0);

  	/* Initialize LCD controller without boost. */
  	SegmentLCD_Init(false);
  
  	// Enable segment blinking.
  	LCD_BlinkEnable(false);         //Set to true to see blink feature.
  	LCD_ContrastSet(0x14);

    
  	while (1){
  		/* Get channels that are pressed, result is or'ed together */
  		    channels_touched = LETOUCH_GetChannelsTouched();

  		    /* Check if any channels are in the touched state. */
  		    if(channels_touched && (channel_no_button()==8)){
  		    	CMU_ClockEnable(cmuClock_HFPER,false);					/*Enabling the HF peripheral clock*/
  		    	CMU_ClockEnable(cmuClock_LCD, false);
  		    	CMU_ClockEnable(cmuClock_LETIMER0, false);					/*Disabling clock to LETIMER*/
  		    }
  		    else if(channels_touched && (channel_no_button()==11)){
  		    	CMU_ClockEnable(cmuClock_HFPER,true);
  		    	CMU_ClockEnable(cmuClock_LETIMER0, true);					/*Enabling clock for LETIMER*/
  		    	CMU_ClockEnable(cmuClock_LCD, true);
  		    }

  		sleep();
  	}
}
