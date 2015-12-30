#include "em_adc.h"
#include "em_system.h"
#include "efm_temperature.h"

/**
 * From Tiny Gecko Datasheet pg 30
 *	TGRAD_ADCTH Thermometer output gradient:
 *		-1.92 mV/¡C
 *		-6.3ADC Codes/¡C
 */
#define TGRAD_ADCTH					(-6.3)


void efm_temperature_init()
{
	ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

	/* Init common settings for both single conversion and scan mode */
	init.timebase = ADC_TimebaseCalc(0);
	/* Might as well finish conversion as quickly as possibly since polling */
	/* for completion. */
	/* Set ADC clock to 7 MHz, use default HFPERCLK */
	init.prescale = ADC_PrescaleCalc(7000000, 0);

	/* WARMUPMODE must be set to Normal according to ref manual before */
	/* entering EM2. In this example, the warmup time is not a big problem */
	/* due to relatively infrequent polling. Leave at default NORMAL, */

	ADC_Init(ADC0, &init);

	/* Init for single conversion use, measure temperature sensor with 1.25 reference. */
	singleInit.reference  = adcRef1V25;
	singleInit.input      = adcSingleInpTemp;
	singleInit.resolution = adcRes12Bit;

	/* The datasheet specifies a minimum aquisition time when sampling vdd/3, but
	 * TODO: what about internet temperature sensor */
	/* 32 cycles should be safe for all ADC clock frequencies */
	singleInit.acqTime = adcAcqTime32;

	ADC_InitSingle(ADC0, &singleInit);
}

uint8_t efm_temperature_get()
{
	uint8_t cal_temp;
	uint32_t gain_calibration_value;
	uint32_t offset_calibration_value;
	uint8_t current_temp;

	/* From 24.3.4.2, page 3.95 of Tiny Gecko Reference Manual
	 * TCELSIUS=CAL_TEMP_0-(ADC0_TEMP_0_READ_1V25- ADC_result) x Vref/(4096 x TGRAD_ADCTH) */

	cal_temp = SYSTEM_GetCalibrationTemperature();

	gain_calibration_value =
			(DEVINFO->ADC0CAL0 & _DEVINFO_ADC0CAL0_1V25_GAIN_MASK)
			>> _DEVINFO_ADC0CAL0_1V25_GAIN_SHIFT;

	offset_calibration_value =
			(DEVINFO->ADC0CAL0 & _DEVINFO_ADC0CAL0_1V25_OFFSET_MASK)
			>> _DEVINFO_ADC0CAL0_1V25_OFFSET_SHIFT;

	current_temp = cal_temp - offset_calibration_value / gain_calibration_value;
	return current_temp;
}
