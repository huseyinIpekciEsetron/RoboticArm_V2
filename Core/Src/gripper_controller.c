/*
 * gripper_controller.c
 *
 *  Created on: Jun 16, 2025
 *      Author: Huseyin
 */

#include "gripper_controller.h"

TIM_HandleTypeDef* gripperTimHandle = NULL;

static float TargetGripperPosition;
static uint16_t GripperPosition;
//static float Integral = 0.0f;
static uint16_t FilteredADC = 0;

static uint16_t DeadBandFilter(uint16_t newValue)
{
	if (fabs((int)newValue - (int)FilteredADC) > ADC_DEADBAND) {
		FilteredADC = newValue;
	}
	return FilteredADC;
}

static uint16_t adc_filter_average(uint32_t new_sample)
{
	static uint16_t adc_buffer[ADC_FILTER_DEPTH];
	static uint8_t adc_index = 0;

    adc_buffer[adc_index++] = DeadBandFilter((uint16_t)new_sample);
    if (adc_index >= ADC_FILTER_DEPTH)
        adc_index = 0;

    uint32_t sum = 0;
    for (int i = 0; i < ADC_FILTER_DEPTH; i++)
        sum += adc_buffer[i];

    return (uint16_t)(2000 - ((sum / ADC_FILTER_DEPTH) - GRIPPER_MIN_POSITION) * (2000 - 1000) / (GRIPPER_MAX_POSITION-GRIPPER_MIN_POSITION));
}

void GripperController_Init(TIM_HandleTypeDef* htim)
{
	gripperTimHandle = htim;
	HAL_TIM_PWM_Start(gripperTimHandle, TIM_CHANNEL_1);

	uint32_t gripperPWM = pwm_us_to_ccr(1400);
	__HAL_TIM_SET_COMPARE(gripperTimHandle, TIM_CHANNEL_1, gripperPWM);

	HAL_Delay(1500); // bekleme sırasında adc okuma yapsın
	TargetGripperPosition = GripperPosition;
}

uint8_t GripperLevel_GetSendGripperPosition() // 0-100
{
	return( (uint8_t)((float)((float)(GripperPosition - GRIPPER_MIN_POSITION)/(float)(GRIPPER_MAX_POSITION-GRIPPER_MIN_POSITION)) * 100.0f) );
}
void GripperLevel_SetGripperPosition(uint32_t *adcData) // GRIPPER_MIN_POSITION - GRIPPER_MAX_POSITION
{
	GripperPosition = adc_filter_average(adcData[0]);
}

void GripperController_Move(float speed)
{
	speed *= MAX_GRIPPER_INCREMENT;
	float delta = speed * MAX_SPEED_MULTIPLIER;

	TargetGripperPosition += delta;

	float diff = TargetGripperPosition - GripperPosition;
	if (diff > 50.0f)
		TargetGripperPosition = GripperPosition + 50.0f;
	else if (diff < -50.0f)
		TargetGripperPosition = GripperPosition - 50.0f;

	TargetGripperPosition = (TargetGripperPosition > MAX_GRIPPER_DUTY_US) ? MAX_GRIPPER_DUTY_US : TargetGripperPosition;
	TargetGripperPosition = (TargetGripperPosition < MIN_GRIPPER_DUTY_US) ? MIN_GRIPPER_DUTY_US : TargetGripperPosition;

	/*float error = (TargetGripperPosition - GripperPosition);
	Integral = Integral + (error) * Dt;

	Integral = (Integral > MAX_INTEGRAL) ? MAX_INTEGRAL : Integral;
	Integral = (Integral < MIN_INTEGRAL) ? MIN_INTEGRAL : Integral;

	uint32_t PWM = 1400 + (Kp * error) + (Ki * Integral);

	PWM = (PWM > MAX_GRIPPER_DUTY_US) ? MAX_GRIPPER_DUTY_US : PWM;
	PWM = (PWM < MIN_GRIPPER_DUTY_US) ? MIN_GRIPPER_DUTY_US : PWM;

	uint32_t gripperPWM = pwm_us_to_ccr(PWM);
*/
	uint32_t gripperPWM = pwm_us_to_ccr(TargetGripperPosition);

	__HAL_TIM_SET_COMPARE(gripperTimHandle, TIM_CHANNEL_1, gripperPWM);

}

uint32_t pwm_us_to_ccr(uint32_t pulse_us)
{
    uint32_t timer_clock = 0;

    // APB1 Prescaler (bits 10:8 of RCC->CFGR)
	uint32_t ppre1 = (RCC->CFGR >> 8) & 0x7;
	uint32_t apb1_clk = HAL_RCC_GetPCLK1Freq();
	if (ppre1 >= 4)  // prescaler != 1
		apb1_clk *= 2;

	// APB2 Prescaler (bits 13:11 of RCC->CFGR)
	uint32_t ppre2 = (RCC->CFGR >> 11) & 0x7;
	uint32_t apb2_clk = HAL_RCC_GetPCLK2Freq();
	if (ppre2 >= 4)
		apb2_clk *= 2;

	// Timer clock belirleme
	if (gripperTimHandle->Instance == TIM1)
		timer_clock = apb2_clk;
	else if (gripperTimHandle->Instance == TIM2 || gripperTimHandle->Instance == TIM3 || gripperTimHandle->Instance == TIM4)
		timer_clock = apb1_clk;
	else
		return 0; // Bilinmeyen timer

    // Timer ayarlarını al
    uint32_t prescaler = gripperTimHandle->Instance->PSC;
    uint32_t arr = gripperTimHandle->Instance->ARR;

    // Timer tick süresi = 1 / (timer_clock / (PSC + 1))
    float tick_time = 1.0f / ((float)timer_clock / (float)(prescaler + 1));

    // Pulse süresi saniye cinsinden
    float pulse_s = (float)pulse_us / 1e6f;

    // CCR değeri = kaç adet tick eder
    uint32_t ccr = (uint32_t)(pulse_s / tick_time);

    // CCR değeri ARR'yi geçmesin
    if (ccr > arr)
        ccr = arr;

    return ccr;
}

