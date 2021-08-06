/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "dbus.h"
#include "chassis_task.h"
#include "chassis_cmd.h"
#include "os_timer.h"
#include "infantry_cmd.h"
#include "board.h"
#include "event_mgr.h"
#include "event.h"
#include "chassis.h"
#include "offline_service.h"
#include "log.h"

struct pid_param chassis_motor_param =
{
    .p = 6.5f,
    .i = 0.1f,
    .max_out = 15000,
    .integral_limit = 500,
};

static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);
static void auto_control_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

static int32_t chassis_angle_broadcast(void *argv);
static void vEnableMotor(void);
static void vLimitedSpeedControl(rc_info_t p_rc_info, uint16_t maxXSpeed, uint16_t maxYSpeed, uint16_t maxWSpeed);
static void vAutoSpeedControl(int ch1, int ch2, int ch3);
static void vClearAutoSpeedData(void);
static int32_t speed_io_led_toggle(void *argc);


struct chassis chassis;
struct rc_device chassis_rc;
struct ahrs_sensor chassis_gyro;

/* chassis speed */
static float vx, vy, wz;
static int status_led_period = 300;
static int iSpeedMod = 0;

/*auto speed*/
static int auto_channel[3];

/* fllow control */
struct pid pid_follow = {0};
float follow_relative_angle;

void chassis_task(void const *argument)
{
    rc_info_t p_rc_info;

    subscriber_t listSubs1;
	subscriber_t listSubs2;
    subscriber_t nolistSubs;

    EventSubscribeInit(&listSubs1, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs1, DBUS_MSG, DBUS_MSG_LEN, 3, chassis_dr16_data_update);

	EventSubscribeInit(&listSubs2, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs2, CONTROL_MSG, CONTROL_MSG_LEN, 3, auto_control_data_update);

    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

    rc_device_register(&chassis_rc, "Chassis RC");
    p_rc_info = rc_device_get_info(&chassis_rc);

    chassis_pid_init(&chassis, "Chassis", chassis_motor_param, DEVICE_CAN2);

    soft_timer_register((soft_timer_callback)chassis_pid_calculate, (void *)&chassis, 5);
    soft_timer_register((soft_timer_callback)chassis_angle_broadcast, (void *)NULL, 10);
	soft_timer_register(speed_io_led_toggle, &status_led_period, 5);
	
    pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

    while (1)
    {
        /* dr16 data update */
        EventMsgProcess(&listSubs1, 0);
        /* gyro data update */
        EventMsgGetLast(&nolistSubs, AHRS_MSG, &chassis_gyro, NULL);
	
		/* control data update*/
		EventMsgProcess(&listSubs2, 0);
		
        chassis_gyro_updata(&chassis, chassis_gyro.yaw * RAD_TO_DEG, chassis_gyro.gz * RAD_TO_DEG);
		
		// 串口控制
		if (rc_device_get_state(&chassis_rc, RC_S1_MID) == E_OK)
		{
			vAutoSpeedControl(auto_channel[0], auto_channel[1], auto_channel[2]);
			iSpeedMod = 3;
		}
		// Dbus控制
		if (rc_device_get_state(&chassis_rc, RC_S1_DOWN) == E_OK)
		{
			if (rc_device_get_state(&chassis_rc, RC_S2_UP) == E_OK)
			{
				vLimitedSpeedControl(p_rc_info, HIGHT_CHASSIS_VX_SPEED, HIGHT_CHASSIS_VY_SPEED, HIGHT_CHASSIS_VW_SPEED);
				iSpeedMod = 2;
			}
			if (rc_device_get_state(&chassis_rc, RC_S2_MID) == E_OK)
			{
				vLimitedSpeedControl(p_rc_info, MID_CHASSIS_VX_SPEED, MID_CHASSIS_VY_SPEED, MID_CHASSIS_VW_SPEED);
				iSpeedMod = 1;
			}
			if (rc_device_get_state(&chassis_rc, RC_S2_DOWN) == E_OK)
			{
				vLimitedSpeedControl(p_rc_info, LOW_CHASSIS_VX_SPEED, LOW_CHASSIS_VY_SPEED, LOW_CHASSIS_VW_SPEED);
				iSpeedMod = 0;
			}
			
			// clear last auto speed data
			vClearAutoSpeedData();
		}
        
		vEnableMotor();
        osDelay(5);
    }
}

/**
  * @brief  send chassis angle to gimbal
  * @param
  * @retval void
  */
int32_t chassis_angle_broadcast(void *argv)
{
    int32_t s_yaw, s_yaw_rate;

    s_yaw = chassis.mecanum.gyro.yaw_gyro_angle * 1000;
    s_yaw_rate = chassis.mecanum.gyro.yaw_gyro_rate * 1000;

    uint8_t data[8];
    data[0] = s_yaw >> 24;
    data[1] = s_yaw >> 16;
    data[2] = s_yaw >> 8;
    data[3] = s_yaw;
    data[4] = s_yaw_rate >> 24;
    data[5] = s_yaw_rate >> 16;
    data[6] = s_yaw_rate >> 8;
    data[7] = s_yaw_rate;

    can1_std_transmit(0x401, data, 8);
    return 0;
}

struct chassis *get_chassis(void)
{
    return &chassis;
}

/**
  * @brief  subscrib dr16 event, update
  * @param
  * @retval void
  */
static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&chassis_rc, pMsgData);
}

/**
  * @brief  follow mode angle update
  * @param
  * @retval void
  */
int32_t follow_angle_info_rcv(uint8_t *buff, uint16_t len)
{
    struct cmd_gimbal_info *info;
    info = (struct cmd_gimbal_info *)buff;
    follow_relative_angle = info->yaw_ecd_angle / 10.0f;
    offline_event_time_update(OFFLINE_GIMBAL_INFO);
    return 0;
}

void set_follow_relative(float val)
{
    follow_relative_angle = val;
}

static void vEnableMotor(void)
{
	/* disable sdk */
	set_chassis_sdk_mode(CHASSIS_SDK_OFF);
	offline_event_disable(OFFLINE_MANIFOLD2_HEART);
	offline_event_disable(OFFLINE_CONTROL_CMD);
	
	offline_event_enable(OFFLINE_CHASSIS_MOTOR1);
	offline_event_enable(OFFLINE_CHASSIS_MOTOR2);
	offline_event_enable(OFFLINE_CHASSIS_MOTOR3);
	offline_event_enable(OFFLINE_CHASSIS_MOTOR4);
}

static void vLimitedSpeedControl(rc_info_t p_rc_info, uint16_t maxXSpeed, uint16_t maxYSpeed, uint16_t maxWSpeed)
{
	vx = (float)p_rc_info->ch2 / 660 * maxXSpeed;
	vy = -(float)p_rc_info->ch1 / 660 * maxYSpeed;
	wz = -(float)p_rc_info->ch3 / 660 * maxWSpeed;
	
	// Max 660
	//log_printf("ch1:%.2f, ch2:%.2f, ch:%.2f\r\n", (float)p_rc_info->ch1, (float)p_rc_info->ch2, (float)p_rc_info->ch3);
	chassis_set_offset(&chassis, 0, 0);
	chassis_set_acc(&chassis, 0, 0, 0);
	chassis_set_speed(&chassis, vx, vy, wz);	
}


int32_t speed_io_led_toggle(void *argc)
{
    static uint32_t led_tick;

    if (get_time_ms() - led_tick > *(int *)argc)
    {
        switch (iSpeedMod)
		{
			// low speed
			case 0:
				LED_R_OFF();
				LED_B_OFF();
				LED_G_TOGGLE();
				break;
			case 1:
				LED_R_OFF();
				LED_G_OFF();
				LED_B_TOGGLE();
				break;
			case 2:
				LED_G_OFF();
				LED_B_OFF();
				LED_R_TOGGLE();
				break;
			case 3:
				LED_G_TOGGLE();
				LED_B_TOGGLE();
				LED_R_TOGGLE();
				break;
		}
        led_tick = get_time_ms();
    }

    return 0;
}
// 接收由shell收到的速度控制信息
static void auto_control_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
	int* channel;
	channel = (int*)pMsgData;
	
	for (int i = 0; i < 3; i++)
	{
		int temp = *(channel + i);
		auto_channel[i] = temp;
	}
	
}

// 设置行驶速度
static void vAutoSpeedControl(int ch1, int ch2, int ch3)
{
	vx = (float)ch2 / 660 * AUTO_CHASSIS_VX_SPEED;
	vy = -(float)ch1 / 660 * AUTO_CHASSIS_VY_SPEED;
	wz = -(float)ch3 / 660 * AUTO_CHASSIS_VW_SPEED;
	
	// Max 660
	//log_printf("ch1:%.2f, ch2:%.2f, ch:%.2f\r\n", (float)p_rc_info->ch1, (float)p_rc_info->ch2, (float)p_rc_info->ch3);
	chassis_set_offset(&chassis, 0, 0);
	chassis_set_acc(&chassis, 0, 0, 0);
	chassis_set_speed(&chassis, vx, vy, wz);	
}

//清除由shell 接收到的速度控制信息
static void vClearAutoSpeedData(void)
{
	for (int i = 0; i < 3; i++)
	{
		auto_channel[i] = 0;
	}
}

