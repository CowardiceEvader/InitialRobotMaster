#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_ZERO_FORCE,                   //chassis will be like no power,��������, ��û�ϵ�����
  CHASSIS_NO_MOVE,                      //chassis will be stop,���̱��ֲ���
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,�����������̸�����̨
  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //chassis will follow chassis yaw angle, usually in engineer,
                                        //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
                                        //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
                                        //���̵��̽Ƕȿ��Ƶ��̣����ڵ���δ�������ǣ��ʶ��Ƕ��Ǽ�ȥ��̨�Ƕȶ��õ���
                                        //����е�������������µ��̵�yaw��pitch��roll�Ƕ� ��chassis_feedback_update������
  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
                                        //���̲�����Ƕȣ��Ƕ��ǿ����ģ������������ٶȻ�
    CHASSIS_OPEN,                         //the value of remote control will mulitiply a value, get current value that will be sent to can bus
    CHASSIS_AUTO_FORWARD_BACK             //auto forward/back mode for fixed testing without remote control input
                                        // ң������ֵ���Ա����ɵ���ֵ ֱ�ӷ��͵�can������
} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. ��chassis_open ģ���£�ң�������Ըñ������͵�can��

// Set to 1 to enable hardcoded motion script mode.
#define CHASSIS_AUTO_FB_ENABLE 1
// If enabled, motion script restarts from first action after the last action.
#define CHASSIS_AUTO_SCRIPT_LOOP_ENABLE 0
// If enabled, auto motion mode will not be overridden by gimbal stop interlock.
#define CHASSIS_AUTO_IGNORE_GIMBAL_STOP 1

// If enabled, chassis CAN current is sent even when all chassis motors are marked offline.
// Useful for bring-up when feedback IDs or detect mapping are still being verified.
#define CHASSIS_FORCE_SEND_WHEN_ALL_OFFLINE 1
// If enabled, request auto shoot after chassis reaches script target.
#define CHASSIS_AUTO_FIRE_ENABLE 1

// Minimum pitch relative angle used by auto fire safety guard (rad).
#define AUTO_FIRE_MIN_PITCH_RELATIVE_ANGLE 0.06f
// Pitch lift gain used to raise muzzle when auto fire is active.
#define AUTO_FIRE_PITCH_LIFT_KP 0.25f
// Max pitch increment command per control cycle during auto lift.
#define AUTO_FIRE_PITCH_LIFT_MAX_ADD 0.0025f



/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

// Runtime debug flags for gimbal->chassis interlock observation.
extern volatile uint8_t chassis_gimbal_stop_requested_flag;
extern volatile uint8_t chassis_gimbal_stop_applied_flag;
extern volatile uint8_t chassis_gimbal_stop_ignored_flag;
extern volatile uint8_t chassis_auto_mode_active_flag;
extern volatile uint8_t chassis_auto_target_reached_flag;
extern volatile uint8_t chassis_auto_fire_enable_flag;

#endif
