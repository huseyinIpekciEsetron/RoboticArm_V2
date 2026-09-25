#include "arm_controller.h"
#include "arm_configuration.h"
#include "inverse_kinematics.h"
#include "arm_state_manager.h"
#include "motor_configuration.h"
#include "motor_control.h"
#include "motor_state.h"
#include "linear_algebra.h"
#include <math.h>

static const float CARTESIAN_SMOOTHING_FACTOR = 0.3f;    // EMA filtre yumuşatma katsayısı
static const float CARTESIAN_MAX_TETHER_ERROR = 8.0f;    // Sanal hedefin koldan maksimum uzaklaşma payı (mm)
static const float CARTESIAN_KP_GAIN = 4.5f;             // Pozisyon hatası düzeltme çarpanı
static const float CARTESIAN_STOP_SPEED_THRESHOLD = 0.1f;// Joystick durma eşiği

static Vec3 virtual_target_position;
static Vec3 smoothed_velocity = { {0.0f, 0.0f, 0.0f} };
static bool is_tracking_active = false;

// Mevcut eklem açılarını hesaplayıp diziye doldurur
static void GetCurrentJointAngles(float joint_angles_out[3]) {
    MotorState joint_states[3] = {MotorState_Get(0), MotorState_Get(1), MotorState_Get(2)};
    for (uint8_t i = 0; i < 3; i++) {
        joint_angles_out[i] = JOINT_DIRECTIONS[i] * (joint_states[i].position - joint_states[i].home_position) * 360.0f / ENCODER_TICKS_PER_REVOLUTION + JOINT_HOME_ANGLES[i];
        joint_angles_out[i] *= (PI / 180.0f);
    }
}

// Takip durumunu ve filtreyi sıfırlar (Joystick bırakıldığında veya yön değiştiğinde)
static void ResetTrackingState(void) {
    is_tracking_active = false;
    smoothed_velocity.data[0] = 0.0f;
    smoothed_velocity.data[1] = 0.0f;
    smoothed_velocity.data[2] = 0.0f;
}

// Yön değişimini kontrol eder ve EMA filtresini uygular
static void ProcessMomentumAndFilter(Vec3 requested_velocity, Vec3 current_tip_position) {
    float dot_prod = LinearAlgebra_CalculateDotProduct(requested_velocity, smoothed_velocity);

    // Eğer yön tersine döndüyse filtreyi ve sanal hedefi sıfırla
    if (dot_prod < 0.0f) {
        ResetTrackingState();
        is_tracking_active = true; // Harekete hemen devam edebilmek için tekrar aktif et
        virtual_target_position = current_tip_position;
    }

    // Hızı Yumuşatma (EMA Filtresi)
    smoothed_velocity.data[0] += (requested_velocity.data[0] - smoothed_velocity.data[0]) * CARTESIAN_SMOOTHING_FACTOR;
    smoothed_velocity.data[1] += (requested_velocity.data[1] - smoothed_velocity.data[1]) * CARTESIAN_SMOOTHING_FACTOR;
    smoothed_velocity.data[2] += (requested_velocity.data[2] - smoothed_velocity.data[2]) * CARTESIAN_SMOOTHING_FACTOR;
}

// Sanal hedefi günceller, koldan kopmasını engeller (Tethering) ve hatayı döndürür
static Vec3 UpdateVirtualTargetAndCalculateError(Vec3 current_tip_position, float dt) {
    // Sanal Hedefi İlerlet
    virtual_target_position.data[0] += smoothed_velocity.data[0] * dt;
    virtual_target_position.data[1] += smoothed_velocity.data[1] * dt;
    virtual_target_position.data[2] += smoothed_velocity.data[2] * dt;

    // Pozisyon hatasını hesapla
    Vec3 position_error = LinearAlgebra_Subtract(virtual_target_position, current_tip_position);
    float error_magnitude = LinearAlgebra_CalculateMagnitude(position_error);

    // Zincirleme (Tethering) Koruması
    if (error_magnitude > CARTESIAN_MAX_TETHER_ERROR) {
        position_error = LinearAlgebra_Scale(LinearAlgebra_Normalize(position_error), CARTESIAN_MAX_TETHER_ERROR);
        virtual_target_position.data[0] = current_tip_position.data[0] + position_error.data[0];
        virtual_target_position.data[1] = current_tip_position.data[1] + position_error.data[1];
        virtual_target_position.data[2] = current_tip_position.data[2] + position_error.data[2];
    }

    return position_error;
}

// Oransal (Kp) kazanç uygulayarak son hızı oluşturur
static Vec3 CalculateClosedLoopCorrection(Vec3 position_error) {
    Vec3 corrected_velocity;
    corrected_velocity.data[0] = smoothed_velocity.data[0] + (CARTESIAN_KP_GAIN * position_error.data[0]);
    corrected_velocity.data[1] = smoothed_velocity.data[1] + (CARTESIAN_KP_GAIN * position_error.data[1]);
    corrected_velocity.data[2] = smoothed_velocity.data[2] + (CARTESIAN_KP_GAIN * position_error.data[2]);
    return corrected_velocity;
}

void ArmController_ApplyJointVelocity(uint8_t joint_index, int32_t velocity) {
  if (joint_index >= NUMBER_OF_MOTORS) return; // Invalid joint ID
  ArmState current_state = ArmStateManager_GetCurrentState();
  if (!(current_state == ARMSTATE_READY )) {
   // return; // Only allow setting joint velocities when the arm is ready (or idle)
  }

  /*
  float joint_angle_in_radians = MotorState_Get(joint_index).position_degree * (PI / 180.0f);
  */

  float joint_angle_in_radians = JOINT_DIRECTIONS[joint_index] * (MotorState_Get(joint_index).position - MotorState_Get(joint_index).home_position) * 360.0f / ENCODER_TICKS_PER_REVOLUTION + JOINT_HOME_ANGLES[joint_index];
  joint_angle_in_radians *= (PI / 180.0f); // Convert degrees to radians
    
  bool is_motion_allowed = ArmController_CheckIfJointMotionAllowed(joint_angle_in_radians, velocity, joint_index);

  if(!is_motion_allowed) { // cartesian için 3 motorun veyLnmışı gerekebilir
	  MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[joint_index], 0); // Stop all joints if motion is not allowed
     return;
  }
  MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[joint_index], JOINT_DIRECTIONS[joint_index] * velocity);
}

void ArmController_StopAllJoints(void) {
  for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
    MotorControl_SetMotorVelocity(MOTOR_CAN_IDS[i], 0);
  }
}

void ArmController_ApplyIKVelocity(Vec3 tip_velocity) {
    float joint_angles[3];
    GetCurrentJointAngles(joint_angles);

    Vec3 modified_tip_velocity = ArmController_ModifyTipVelocity(joint_angles[0], joint_angles[1], joint_angles[2], tip_velocity);
    Vec3 joint_velocities = InverseKinematics_ComputeJointVelocities(joint_angles[0], joint_angles[1], joint_angles[2], LENGTH_OF_FIRST_ARM, LENGTH_OF_SECOND_ARM, modified_tip_velocity);

    // Limit ve güvenlik kontrolleri
    for (uint8_t i = 0; i < 3; i++) {
        bool is_motion_allowed = ArmController_CheckIfJointMotionAllowed(joint_angles[i], joint_velocities.data[i], i);
        if (!is_motion_allowed) {
            ArmController_StopAllJoints();
            return;
        }
    }

    // Hızları motor tick formatına çevir ve uygula
    for (uint8_t i = 0; i < 3; i++) {
        int32_t final_velocity = (int32_t)((joint_velocities.data[i] * 180.0f * 101.0f * 100.0f) / (3.14159265358979323846f * 360.0f));
        ArmController_ApplyJointVelocity(i, final_velocity);
    }
}

Vec3 ArmController_ModifyTipVelocity(float theta0, float theta1, float theta2, Vec3 desired_tip_velocity) {
  Vec3 current_tip_position = ArmController_CalculateTipPosition(theta0, theta1, theta2);
  Vec3 modified_velocity = desired_tip_velocity;

  if (LinearAlgebra_CalculateMagnitude(current_tip_position) > (LENGTH_OF_FIRST_ARM + LENGTH_OF_SECOND_ARM) * 0.9f) {
    // If the tip is near the boundary of the reachable area, block outgoing velocity requests
    Vec3 normal_to_boundary = LinearAlgebra_Normalize(current_tip_position); // Normal vector pointing outward from the center of the arm
    float v_dot_n = LinearAlgebra_CalculateDotProduct(desired_tip_velocity, normal_to_boundary);
    if (v_dot_n > 0.0f) {
      modified_velocity.data[0] = 0.0f;
      modified_velocity.data[1] = 0.0f;
      modified_velocity.data[2] = 0.0f; // No motion
      return modified_velocity;
    }
  }

  for (int i = 0; i < NUMBER_OF_BLOCKED_SPHERES; ++i) {
    const SphericalVolume* sphere = &BLOCKED_SPHERICAL_VOLUMES[i];

    Vec3 tip_to_center = LinearAlgebra_Subtract(current_tip_position, sphere->center_position);
    float distance_to_center = LinearAlgebra_CalculateMagnitude(tip_to_center);

    if (distance_to_center < (sphere->radius + sphere->viscous_thickness)) {
      // Inside viscous volume only → halve speed, keep direction
      modified_velocity = LinearAlgebra_Scale(modified_velocity, 0.5f);
      if (distance_to_center <= sphere->radius) {
      Vec3 normal_to_surface = LinearAlgebra_Normalize(tip_to_center);
        float v_dot_n = LinearAlgebra_CalculateDotProduct(modified_velocity, normal_to_surface);
        float speed = LinearAlgebra_CalculateMagnitude(modified_velocity);
        // Inside hard volume
        if (v_dot_n < 0.0f) {
          // Velocity points toward center → project onto tangent plane
          Vec3 tangent = LinearAlgebra_ProjectOnPlane(modified_velocity, normal_to_surface);
          if (LinearAlgebra_CalculateMagnitude(tangent) > 1e-5f) {
            modified_velocity = LinearAlgebra_Scale(LinearAlgebra_Normalize(tangent), speed);
          } else {
            modified_velocity.data[0] = 0.0f;
            modified_velocity.data[1] = 0.0f;
            modified_velocity.data[2] = 0.0f; // Too small tangent = no motion
          }
        }
      }
    }
    // Else: velocity points out or is tangent → keep as is
  }
  return modified_velocity;
}

Vec3 ArmController_CalculateTipPosition(float theta0, float theta1, float theta2) {
  float sin_theta0 = sinf(theta0);
  float cos_theta0 = cosf(theta0);
  float sin_theta1 = sinf(theta1);
  float cos_theta1 = cosf(theta1);

  float theta2_prime = theta1 + theta2 - PI; // Combined angle for the second joint
  float cos_theta2_prime = cosf(theta2_prime);
  float sin_theta2_prime = sinf(theta2_prime);

  float r = LENGTH_OF_FIRST_ARM * cos_theta1 + LENGTH_OF_SECOND_ARM * cos_theta2_prime; // Effective length in the radial direction

  Vec3 tip_position;

  tip_position.data[0] = r * cos_theta0; // X
  tip_position.data[1] = r * sin_theta0; // Y
  tip_position.data[2] = LENGTH_OF_FIRST_ARM * sin_theta1 + LENGTH_OF_SECOND_ARM * sin_theta2_prime; //Z

  return tip_position;
}

bool ArmController_CheckIfJointMotionAllowed(float joint, float joint_velocity, uint8_t index) {
  // Check if the joint velocities are within the allowed limits
  /*
  if (fabs(joint_0_velocity) > MAX_JOINT_VELOCITY || fabs(joint_1_velocity) > MAX_JOINT_VELOCITY || fabs(joint_2_velocity) > MAX_JOINT_VELOCITY) {
    return false; // Joint motion not allowed due to velocity limits
  }
  */
	switch(index)
	{
		case 0:
			if ( (joint > (JOINT_HOME_ANGLES[0] - JOINT_LIMIT_OFFSETS[0]) * 3.14159265358979323846f / 180.0f && joint_velocity > 0) \
					|| (joint < (-JOINT_HOME_ANGLES[0] + JOINT_LIMIT_OFFSETS[0]) * 3.14159265358979323846f / 180.0f && joint_velocity < 0) )
				return false;
			break;
		case 1:
			if( (joint > (200.0f - JOINT_LIMIT_OFFSETS[1]) * 3.14159265358979323846f / 180.0f && joint_velocity > 0) \
					|| (joint < (-20.0f + JOINT_LIMIT_OFFSETS[1]) * 3.14159265358979323846f / 180.0f && joint_velocity < 0) )
				return false;
			break;
		case 2:
			if((joint > (250.0f - JOINT_LIMIT_OFFSETS[2]) * 3.14159265358979323846f / 180.0f && joint_velocity > 0) \
					|| (joint < (JOINT_HOME_ANGLES[2] + JOINT_LIMIT_OFFSETS[2]) * 3.14159265358979323846f / 180.0f && joint_velocity < 0) )
				return false;
			break;
		default:
			break;
	}

  // Check if the joint angles are within the allowed limits
  /*if (
    (joint_0 > (JOINT_HOME_ANGLES[0] - JOINT_LIMIT_OFFSETS[0]) * 3.14159265358979323846f / 180.0f && joint_0_velocity > 0) ||
    (joint_0 < (-JOINT_HOME_ANGLES[0] + JOINT_LIMIT_OFFSETS[0]) * 3.14159265358979323846f / 180.0f && joint_0_velocity < 0) ||
    (joint_1 > (JOINT_HOME_ANGLES[1] - JOINT_LIMIT_OFFSETS[1]) * 3.14159265358979323846f / 180.0f && joint_1_velocity > 0) ||
    (joint_1 < (0.0f + JOINT_LIMIT_OFFSETS[1]) * 3.14159265358979323846f / 180.0f && joint_1_velocity < 0) ||
    (joint_2 > (180.0f - JOINT_LIMIT_OFFSETS[2]) * 3.14159265358979323846f / 180.0f && joint_2_velocity > 0) ||
    (joint_2 < (JOINT_HOME_ANGLES[2] + JOINT_LIMIT_OFFSETS[2]) * 3.14159265358979323846f / 180.0f && joint_2_velocity < 0)
  ) {
    return false; // Joint motion not allowed due to angle limits
  }*/

  return true; // Joint motion is allowed
}

bool ArmController_MoveTipToPosition(Vec3 target_position, float speed, bool will_stop_when_arrived) {
  bool is_arrived = false;
  MotorState joint_states[3] = {MotorState_Get(0), MotorState_Get(1), MotorState_Get(2)};

  float joint_angles[3] = {0.0f};
  for (uint8_t i = 0; i < 3; i++) {
    joint_angles[i] = JOINT_DIRECTIONS[i] * (joint_states[i].position - joint_states[i].home_position) * 360.0f / ENCODER_TICKS_PER_REVOLUTION + JOINT_HOME_ANGLES[i];
    joint_angles[i] *= (PI / 180.0f); // Convert degrees to radians
  }

  Vec3 current_position = ArmController_CalculateTipPosition(joint_angles[0], joint_angles[1], joint_angles[2]);
  Vec3 difference_vector = LinearAlgebra_Subtract(target_position, current_position);
  Vec3 direction = LinearAlgebra_Normalize(difference_vector);
  float distance = LinearAlgebra_CalculateMagnitude(difference_vector);
  // if (distance <= POSITION_CONTROL_RADIUS * 2) speed = speed / 2;
  if (distance <= POSITION_CONTROL_RADIUS) {
    if (will_stop_when_arrived) speed = 0.0f;
    is_arrived = true;
  }
  Vec3 velocity;
  velocity.data[0] = direction.data[0] * speed;
  velocity.data[1] = direction.data[1] * speed;
  velocity.data[2] = direction.data[2] * speed;
  ArmController_ApplyIKVelocity(velocity);
  return is_arrived;
}


// Dışarıdan robotu kartezyen sürmek için
// tip_velocity: Joystick'ten gelen X,Y,Z hız isteği
// dt: Döngünün çalışma süresi (Örn: 10ms loop için 0.01f)
void ArmController_DriveCartesianClosedLoop(Vec3 tip_velocity, float dt) {
    // 1. Mevcut pozisyonu hesapla
    float joint_angles[3];
    GetCurrentJointAngles(joint_angles);
    Vec3 current_tip_position = ArmController_CalculateTipPosition(joint_angles[0], joint_angles[1], joint_angles[2]);

    // 2. Joystick durma kontrolü
    float tip_speed = LinearAlgebra_CalculateMagnitude(tip_velocity);
    if (tip_speed < CARTESIAN_STOP_SPEED_THRESHOLD) {
        ResetTrackingState();
        ArmController_StopAllJoints();
        return;
    }

    // 3. Başlangıç durumu
    if (!is_tracking_active) {
        virtual_target_position = current_tip_position;
        is_tracking_active = true;
    }

    // 4. Modüler işlemleri sırasıyla çağır
    ProcessMomentumAndFilter(tip_velocity, current_tip_position);
    Vec3 position_error = UpdateVirtualTargetAndCalculateError(current_tip_position, dt);
    Vec3 corrected_velocity = CalculateClosedLoopCorrection(position_error);

    // 5. Düzeltilmiş hızı IK'ya gönder
    ArmController_ApplyIKVelocity(corrected_velocity);
}

