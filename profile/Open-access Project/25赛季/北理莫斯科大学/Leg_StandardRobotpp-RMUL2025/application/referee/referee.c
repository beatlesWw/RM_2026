/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.c/h
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note       裁判系统通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "referee.h"

#include "CRC8_CRC16.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "protocol.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define REFEREE_TIMEOUT 20  //裁判系统超时时间
bool referee_is_offline = false;
uint32_t referee_online_time = 0;


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_status;                                           //0x0001
game_result_t game_result;                                           //0x0002
game_robot_HP_t game_robot_HP;                                       //0x0003

event_data_t event_data;                                             //0x0101
referee_warning_t referee_warning;                                   //0x0104
dart_info_t dart_info;                                               //0x0105

robot_status_t robot_status;                                         //0x0201
power_heat_data_t power_heat_data;                                   //0x0202
robot_pos_t robot_pos;                                               //0x0203
buff_t buff;                                                         //0x0204
hurt_data_t hurt_data;                                               //0x0206
shoot_data_t shoot_data;                                             //0x0207
projectile_allowance_t projectile_allowance;                         //0x0208
rfid_status_t rfid_status;                                           //0x0209
dart_client_cmd_t dart_client_cmd;                                   //0x020A
ground_robot_position_t ground_robot_position;                       //0x020B
radar_mark_data_t radar_mark_data;                                   //0x020C
sentry_info_t sentry_info;                                           //0x020D
radar_info_t radar_info;                                             //0x020E


robot_interaction_data_t robot_interaction_data;                     //0x0301
custom_robot_data_t custom_robot_data;                               //0x0302
map_command_t map_command;                                           //0x0303
remote_control_t remote_control;                                     //0x0304



void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

    memset(&event_data, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0 , sizeof(dart_info_t));

    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));

    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&remote_control, 0, sizeof(remote_control_t));
}

void referee_data_solve(uint8_t * frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    buzzer_on(100, 300);

    switch (cmd_id) {
        case GAME_STATUS_CMD_ID: {
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_RESULT_CMD_ID: {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_ROBOT_HP_CMD_ID: {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            referee_online_time = HAL_GetTick();
        } break;

        case EVENT_DATA_CMD_ID: {
            memcpy(&event_data, frame + index, sizeof(event_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case REFEREE_WARNING_CMD_ID: {
            memcpy(
                &referee_warning, frame + index, sizeof(referee_warning_t));
            referee_online_time = HAL_GetTick();
        } break;
        case DART_INFO_CMD_ID: {
            memcpy(
                &dart_info, frame + index,
                sizeof(dart_info_t));
            referee_online_time = HAL_GetTick();
        } break;
        
        case ROBOT_STATUS_CMD_ID: {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case POWER_HEAT_DATA_CMD_ID: {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_POS_CMD_ID: {
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
            referee_online_time = HAL_GetTick();
        } break;
        case BUFF_CMD_ID: {
            memcpy(&buff, frame + index, sizeof(buff_t));
            referee_online_time = HAL_GetTick();
        } break;
        case HURT_DATA_CMD_ID: {
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SHOOT_DATA_CMD_ID: {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case PROJECTILE_ALLOWANCE_CMD_ID: {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RFID_STATUS_CMD_ID: {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case DART_CLIENT_CMD_CMD_ID: {
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GROUND_ROBOT_POSITION_CMD_ID: {
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RADAR_MARK_DATA_CMD_ID: {
            memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SENTRY_INFO_CMD_ID: {
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RADAR_INFO_CMD_ID: {
            memcpy(&radar_info, frame + index, sizeof(radar_info_t));
            referee_online_time = HAL_GetTick();
        } break;

        case ROBOT_INTERACTION_DATA_CMD_ID: {
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case CUSTOM_ROBOT_DATA_CMD_ID: {
            memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case MAP_COMMAND_CMD_ID: {
            memcpy(&map_command, frame + index, sizeof(map_command_t));
            referee_online_time = HAL_GetTick();
        } break;
        case REMOTE_CONTROL_CMD_ID: {
            memcpy(&remote_control, frame + index, sizeof(remote_control_t));
            referee_online_time = HAL_GetTick();
        } break;
        default: {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 * power, fp32 * buffer)
{
    *power = power_heat_data.buffer_energy;
    *buffer = power_heat_data.buffer_energy;
}

uint8_t get_robot_id(void) { return robot_status.robot_id; }

void get_shoot_heat0_limit_and_heat0(uint16_t * heat0_limit, uint16_t * heat0)
{
    *heat0_limit = robot_status.shooter_barrel_heat_limit;
    *heat0 = power_heat_data.shooter_17mm_1_barrel_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t * heat1_limit, uint16_t * heat1)
{
    *heat1_limit = robot_status.shooter_barrel_heat_limit;
    *heat1 = power_heat_data.shooter_17mm_2_barrel_heat;  // 第 2 个 17mm 发射机构的枪口热量
}


void get_shoot_heat42_limit_and_heat42(uint16_t *heat_limit, uint16_t *heat)
{
    *heat_limit = robot_status.shooter_barrel_heat_limit;
    *heat = power_heat_data.shooter_42mm_barrel_heat;
}

/**
 * @brief 反馈机器人颜色
 * @param  none
 * @return 0为红色,1为蓝色,2为未知
 */
uint8_t get_team_color(void)  // 谨防“哨兵在打我”
{
    switch (robot_status.robot_id) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
            return 0;
        case 101:
        case 102:
        case 103:
        case 104:
        case 105:
        case 106:
        case 107:
        case 108:
        case 109:
        case 110:
        case 111:
            return 1;
        default:
            return 2;
    }
}

uint16_t get_shoot_heat(void)  // 双枪管哨兵
{
    if (power_heat_data.shooter_17mm_1_barrel_heat > power_heat_data.shooter_17mm_2_barrel_heat) {
        return power_heat_data.shooter_17mm_1_barrel_heat;
    } else {
        return power_heat_data.shooter_17mm_2_barrel_heat;
    }
}

custom_robot_data_t * GetCustomControllerDataPoint(void) { return &custom_robot_data; }

/*========== API ==========*/

inline bool GetRefereeState(void) { 
    uint32_t current_time = HAL_GetTick();
    return current_time - referee_online_time < REFEREE_TIMEOUT && current_time>REFEREE_TIMEOUT;
}

/**
 * @brief 获取自定义控制器数据
 * @param  index 数据索引
 * @return float 数据
 */
inline float GetCustomControllerPos(uint8_t index){
    float data = 0;
    memcpy(&data, &custom_robot_data.data[index * 4], 4);
    return data;
    // return *((float *)(&custom_robot_data.data[index * 4]));
}
// /**
//  * @brief 获取事件数据
//  * @param  无
//  * @return uint32_t 
//  */
// inline event_data_t GetEventData(void){return event_data;}

// /**
//  * @brief 获取机器人血量数据
//  * @param  无
//  * @return game_robot_HP_t
//  */
// inline game_robot_HP_t GetGameRobotHP(void){return game_robot_HP;}

// /**
//  * @brief 获取比赛信息
//  * @param  无
//  * @return game_status_t
//  */
// inline game_status_t GetGameStatus(void){return game_status;}

// /**
//  * @brief 获取地面机器人位置信息
//  * @param  无
//  * @return ground_robot_position_t
//  */
// inline ground_robot_position_t GetGroundRobotPosition(void){return ground_robot_position;}

// /**
//  * @brief 获取RFID信息
//  * @param  无
//  * @return rfid_status_t
//  */
// inline rfid_status_t GetRFIDStatus(void){return rfid_status;}


// /**
//  * @brief 获取机器人信息
//  * @param  无
//  * @return robot_status_t
//  */
// inline robot_status_t GetRobotStatus(void){return robot_status;}

// /**
//  * @brief 获取能量热量信息
//  * @param  无
//  * @return power_heat_data_t
//  */
// inline power_heat_data_t GetPoweHeatData(void){return power_heat_data;}

// /**
//  * @brief 获取机器人位姿信息
//  * @param  无
//  * @return robot_pos_t
//  */
// inline robot_pos_t GetRobotPos(void){return robot_pos;}

// /**
//  * @brief 获取受伤信息
//  * @param  无
//  * @return hurt_data_t
//  */
// inline hurt_data_t GetHurtData(void){return hurt_data;}

// /**
//  * @brief 获取允许发弹量信息
//  * @param  无
//  * @return hurt_data_t
//  */
// inline projectile_allowance_t GetProjectiliAllowance(void){return projectile_allowance;}

// /**
//  * @brief 获取buff信息
//  * @param  无
//  * @return buff_t
//  */
// inline buff_t GetBuff(void){return buff;}
/*------------------------------ End of File ------------------------------*/
