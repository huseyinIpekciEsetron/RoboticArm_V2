#include "motor_configuration.h"


int MotorConfiguration_FindIndexOfMotor(int id)
{
  int index = 0;
  while (index < NUMBER_OF_MOTORS && MOTOR_CAN_IDS[index] != id) ++index;
  return (index == NUMBER_OF_MOTORS ? -1 : index);
}

float convertEncoderTicksToDegrees(int32_t position, uint8_t index)
{
  float posToDeg = (JOINT_DIRECTIONS[index] * (position - MotorState_Get(index).home_position) * 360.0f / ENCODER_TICKS_PER_REVOLUTION + JOINT_HOME_ANGLES[index]);
  posToDeg = fmodf(posToDeg, 360.0f);
  if (posToDeg < -180.0f) posToDeg += 360.0f;
  if (posToDeg > 180.0f) posToDeg -= 360.0f;
  return posToDeg;
}
