// motor_state.c
#include "motor_state.h"
#include "motor_configuration.h"
#include "stm32f1xx_hal.h"
#include <string.h>

bool MotorState_MechanicalBreak[4] = {false, false, false, false};
static MotorState motor_states[NUMBER_OF_MOTORS];

MotorState MotorState_Get(uint8_t index) {
    MotorState dummy = {0};
    if (index >= NUMBER_OF_MOTORS) return dummy;

    MotorState result;
    __disable_irq();           		// Begin critical section
    result = motor_states[index]; // Struct copy
    __enable_irq();            		// End critical section

    return result;
}

// Motor state'i offset'e göre günceller. Offset'ler header'da define'lar ile tanımlanmıştır.
void MotorState_Set(uint8_t index, size_t offset, const void* data, size_t size) {
	if (index >= NUMBER_OF_MOTORS || !data) return;

	__disable_irq();
	// You cast the pointer to uint8_t* (aka unsigned char*), which is a pointer to a single byte.
	// Because only uint8_t* allows you to do byte-wise pointer arithmetic.
	// If you didn't cast, adding an offset to a MotorState* would jump offset * sizeof(MotorState) bytes — not what you want!
	memcpy((uint8_t*)&motor_states[index] + offset, data, size);
	__enable_irq();
}

void MotorState_SetHomeToCurrentPosition(uint8_t id) {
	int32_t position = MotorState_Get(id).position;
	MotorState_Set(id, MOTOR_STATE_HOME_POSITION, &position, sizeof(position));
}
