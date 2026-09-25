/*
 * homing.c
 *
 *  Created on: Jun 17, 2025
 *      Author: Huseyin
 */
#include "homing.h"
static homing_Status HomingState = 0;

static float prev_angle_deg = 0.0f;  // Bir önceki açı
static float angular_velocity_deg_s = 0.0f; // Çıktı: °/s

/*
 * GPIO EXTI kullanımında bu fonksiyon kullanılıyordu
 * hata olarak başlangıç konumundaysa high olarak başlayıp risinge geçemediği için kullanılmayacak
 */
 void Homing_HallEffectDetecter(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_12:
			ArmStateManager_OnHomeSensorTriggered(MOTOR_2);
			break;
		case GPIO_PIN_13:
			ArmStateManager_OnHomeSensorTriggered(MOTOR_3);
			break;
		case GPIO_PIN_4:
			ArmStateManager_OnHomeSensorTriggered(MOTOR_1);
			break;
	}

}

 /*
  * çözüm olarak bu yöntem tercih edildi ama mıknatısların yönü değiştirilmeli
  * çünkü sensör high olarak başlayacak bu durumda sınıra geldiğinde önce high sonra low yapacak şekilde mıknatısları yerleştirmek lazım
  */
void Homing_FirstStageHallEffectDetect()
{
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
		ArmStateManager_OnHomeSensorTriggered(MOTOR_3);
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
		ArmStateManager_OnHomeSensorTriggered(MOTOR_2);
}

void Homing_SecondStageHallEffectDetect()
{
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		ArmStateManager_OnHomeSensorTriggered(MOTOR_1);
}

bool Homing_FirstStageMotorCurrentLimitControl()
{
	if( (MOTOR_CURRENT(MOTOR_2) > MAX_MOTOR2_CURRENT) || (MOTOR_CURRENT(MOTOR_3) > MAX_MOTOR3_CURRENT) )
		return true;
	return false;
}

bool Homing_SecondStageMotorCurrentLimitControl()
{
	if(MOTOR_CURRENT(MOTOR_1) > MAX_MOTOR1_CURRENT)
		return true;
	return false;
}
homing_Status Homing_getHomingState()
{
	return HomingState;
}
void Homing_setHomingState(homing_Status homeState)
{
	HomingState = homeState;
}


void Homing_update_angular_velocity(float current_angle_deg, float dt_s) {
    float diff = current_angle_deg - prev_angle_deg;

    // 360° sarma düzeltmesi (ör. 359° → 0° geçişi)
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }

    angular_velocity_deg_s = diff / dt_s; // °/s hesapla
    prev_angle_deg = current_angle_deg;
}
