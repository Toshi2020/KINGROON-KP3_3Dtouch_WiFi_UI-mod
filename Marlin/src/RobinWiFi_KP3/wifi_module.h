/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#ifdef __cplusplus
  extern "C" {
#endif

///#include "../../../inc/MarlinConfigPre.h"
#include "../inc/MarlinConfigPre.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_RX_BUFFER_SIZE   1024
#define UART_FIFO_BUFFER_SIZE 1024

#define WIFI_DECODE_TYPE      1

#define IP_DHCP_FLAG          1

#define WIFI_AP_NAME          "TP-LINK_MKS"
#define WIFI_KEY_CODE         "makerbase"

#define IP_ADDR               "192.168.3.100"
#define IP_MASK               "255.255.255.0"
#define IP_GATE               "192.168.3.1"
#define IP_DNS                "192.168.3.1"

#define AP_IP_DHCP_FLAG       1
#define AP_IP_ADDR            "192.168.3.100"
#define AP_IP_MASK            "255.255.255.0"
#define AP_IP_GATE            "192.168.3.1"
#define AP_IP_DNS             "192.168.3.1"
#define IP_START_IP           "192.168.3.1"
#define IP_END_IP             "192.168.3.255"

#define UDISKBUFLEN           1024

typedef enum {
  udisk_buf_empty = 0,
  udisk_buf_full,
} UDISK_DATA_BUFFER_STATE;

/// RAMの使用量が94%以上ではmarlinが不安定になる(ハングして再起動する)
///#define TRANS_RCV_FIFO_BLOCK_NUM  14	// KP3変更：メモリ削減
#define TRANS_RCV_FIFO_BLOCK_NUM  2	// 4に上げても転送時間は変わらない

typedef struct {
  bool receiveEspData;
  unsigned char *bufferAddr[TRANS_RCV_FIFO_BLOCK_NUM];
  unsigned char *p;
  UDISK_DATA_BUFFER_STATE state[TRANS_RCV_FIFO_BLOCK_NUM];
  unsigned char read_cur;
  unsigned char write_cur;
} WIFI_DMA_RCV_FIFO;

typedef struct {
  uint8_t flag; // 0x0: no error;  0x01: error
  uint32_t start_tick; // error start time
  uint32_t now_tick;
} WIFI_TRANS_ERROR;

extern volatile WIFI_TRANS_ERROR wifiTransError;

typedef struct {
  char ap_name[32];   // wifi-name
  char keyCode[64]; // wifi password
  int decodeType;
  int baud;
  int mode;
} WIFI_PARA;

typedef struct {
  char state;
  char hostUrl[96];
  int port;
  char id[21];
} CLOUD_PARA;

typedef struct {
  char dhcp_flag;
  char ip_addr[16];
  char mask[16];
  char gate[16];
  char dns[16];

  char dhcpd_flag;
  char dhcpd_ip[16];
  char dhcpd_mask[16];
  char dhcpd_gate[16];
  char dhcpd_dns[16];
  char start_ip_addr[16];
  char end_ip_addr[16];
} IP_PARA;

typedef enum {
  WIFI_NOT_CONFIG,
  WIFI_CONFIG_MODE,
  WIFI_CONFIG_DHCP,
  WIFI_CONFIG_AP,
  WIFI_CONFIG_IP_INF,
  WIFI_CONFIG_DNS,
  WIFI_CONFIG_TCP,
  WIFI_CONFIG_SERVER,
  WIFI_CONFIG_REMOTE_PORT,
  WIFI_CONFIG_BAUD,
  WIFI_CONFIG_COMMINT,
  WIFI_CONFIG_OK,
  WIFI_GET_IP_OK,
  WIFI_RECONN,
  WIFI_CONNECTED,
  WIFI_WAIT_TRANS_START,
  WIFI_TRANS_FILE,
  WIFI_CONFIG_DHCPD,
  WIFI_COFIG_DHCPD_IP,
  WIFI_COFIG_DHCPD_DNS,
  WIFI_EXCEPTION,
} WIFI_STATE;

typedef enum {
  TRANSFER_IDLE,
  TRANSFERRING,
  TRANSFER_STORE,
} TRANSFER_STATE;
extern volatile TRANSFER_STATE esp_state;

typedef struct {
  char buf[20][80];
  int rd_index;
  int wt_index;
} QUEUE;

typedef enum {
  WIFI_PARA_SET,      // 0x0:net parameter
  WIFI_PRINT_INF,     // 0x1:print message
  WIFI_TRANS_INF,     // 0x2:Pass through information
  WIFI_EXCEP_INF,     // 0x3:Exception information
  WIFI_CLOUD_CFG,     // 0x4:cloud config
  WIFI_CLOUD_UNBIND,  // 0x5:Unbind ID
} WIFI_RET_TYPE;

typedef struct {
  uint32_t uart_read_point;
  uint32_t uart_write_point;
  //uint8_t uartTxBuffer[UART_FIFO_BUFFER_SIZE];
} SZ_USART_FIFO;

#define WIFI_GCODE_BUFFER_LEAST_SIZE    96
#define WIFI_GCODE_BUFFER_SIZE  (WIFI_GCODE_BUFFER_LEAST_SIZE * 3)
typedef struct {
  uint8_t wait_tick;
  uint8_t Buffer[WIFI_GCODE_BUFFER_SIZE];
  uint32_t r;
  uint32_t w;
} WIFI_GCODE_BUFFER;

extern volatile WIFI_STATE wifi_link_state;
extern WIFI_PARA wifiPara;
extern IP_PARA ipPara;
extern CLOUD_PARA cloud_para;

extern WIFI_GCODE_BUFFER espGcodeFifo;

uint32_t getWifiTick();
uint32_t getWifiTickDiff(int32_t lastTick, int32_t curTick);

void mks_esp_wifi_init();
extern int cfg_cloud_flag;
int send_to_wifi(uint8_t *buf, int len);
void wifi_looping();
int raw_send_to_wifi(uint8_t *buf, int len);
int package_to_wifi(WIFI_RET_TYPE type, uint8_t *buf, int len);
void get_wifi_list_command_send();
void get_wifi_commands();
int readWifiBuf(int8_t *buf, int32_t len);
void mks_wifi_firmware_update();
int usartFifoAvailable(SZ_USART_FIFO *fifo);
int readUsartFifo(SZ_USART_FIFO *fifo, int8_t *buf, int32_t len);
void esp_port_begin(uint8_t interrupt);

/// KP3追加 MKS WiFiモジュールのイニシャライズ
void mks_wifi_init();

/// KP3追加(draw_print_file.hより移植) ///////////////////////////////////////
#define FILE_NUM 6
#define SHORT_NAME_LEN 13
#define NAME_CUT_LEN 23

///#define MAX_DIR_LEVEL  10
#define MAX_DIR_LEVEL  3	/// RAM節約。パス+ファイル名で40文字

typedef struct {
  char file_name[FILE_NUM][SHORT_NAME_LEN * MAX_DIR_LEVEL + 1];
  char curDirPath[SHORT_NAME_LEN * MAX_DIR_LEVEL + 1];
  char long_name[FILE_NUM][SHORT_NAME_LEN * 2 + 1];
  bool IsFolder[FILE_NUM];
  char Sd_file_cnt;
  char sd_file_index;
  char Sd_file_offset;
} LIST_FILE;
extern LIST_FILE list_file;

/// KP3追加(draw_printing.hより移植) /////////////////////////////////////////
enum {
  IDLE,
  WORKING,
  PAUSING,
  PAUSED,
  REPRINTING,
  REPRINTED,
  RESUMING,
  STOP
};

/// KP3追加(draw_ui.hより移植) ///////////////////////////////////////////////
#define ESP_WIFI          0x02
#define AP_MODEL          0x01
#define STA_MODEL         0x02

#define FILE_SYS_USB      0
#define FILE_SYS_SD       1

#define TICK_CYCLE 1
typedef struct {
///  uint32_t  spi_flash_flag;
///  uint8_t   disp_rotation_180;
///  bool      multiple_language;
///  uint8_t   language;
///  uint8_t   leveling_mode;
///  bool      from_flash_pic;
///  bool      finish_power_off;
///  bool      pause_reprint;
  uint8_t   wifi_mode_sel;
  uint8_t   fileSysType;
  uint8_t   wifi_type;
  bool      cloud_enable,
            encoder_enable;
///  xy_int_t  trammingPos[5];
///  int       filamentchange_load_length,
///            filamentchange_load_speed,
///            filamentchange_unload_length,
///            filamentchange_unload_speed;
///  celsius_t filament_limit_temp;
///  float     pausePosX, pausePosY, pausePosZ;
  uint32_t  curFilesize;
} CFG_ITMES;

typedef struct UI_Config_Struct {
///  uint8_t curTempType:1,
///          extruderIndex:3,
///          stepHeat:4,
///          extruderIndexBak:4;
  bool    leveling_first_time:1,
          para_ui_page:1,
          configWifi:1,
          command_send:1,
          filament_load_heat_flg:1,
          filament_heat_completed_load:1,
          filament_unload_heat_flg:1,
          filament_heat_completed_unload:1,
          filament_loading_completed:1,
          filament_unloading_completed:1,
          filament_loading_time_flg:1,
          filament_unloading_time_flg:1;
  uint8_t wifi_name[32];
  uint8_t wifi_key[64];
  uint8_t cloud_hostUrl[96];
  // Extruder Steps distances (mm)
///  uint8_t extruStep;
///  static constexpr uint8_t eStepMin =  1,
///                           eStepMed =  5,
///                           eStepMax = 10;
  // Extruder speed (mm/s)
///  uint8_t extruSpeed;
///  static constexpr uint8_t eSpeedH = 20,
///                           eSpeedN = 10,
///                           eSpeedL =  1;
  uint8_t print_state;
///  uint8_t stepPrintSpeed;
///  uint8_t waitEndMoves;
///  uint8_t dialogType;
///  uint8_t F[4];
///  uint8_t filament_rate;
///  uint16_t moveSpeed;
  uint16_t cloud_port;
///  uint16_t moveSpeed_bak;
///  uint32_t totalSend;
///  uint32_t filament_loading_time,
///           filament_unloading_time,
///           filament_loading_time_cnt,
///           filament_unloading_time_cnt;
///  float move_dist;
//  celsius_t hotendTargetTempBak;
///  float current_x_position_bak,
///        current_y_position_bak,
///        current_z_position_bak,
///        current_e_position_bak;
} UI_CFG;

/// KP3追加(draw_wifi_list.hより移植) ////////////////////////////////////////
#define NUMBER_OF_PAGE 5

#define WIFI_TOTAL_NUMBER 20
#define WIFI_NAME_BUFFER_SIZE 33

typedef struct {
  int8_t getNameNum;
  int8_t nameIndex;
  int8_t currentWifipage;
  int8_t getPage;
  int8_t RSSI[WIFI_TOTAL_NUMBER];
  uint8_t wifiName[WIFI_TOTAL_NUMBER][WIFI_NAME_BUFFER_SIZE];
  uint8_t wifiConnectedName[WIFI_NAME_BUFFER_SIZE];
} WIFI_LIST;
extern WIFI_LIST wifi_list;

typedef struct list_menu_disp {
  const char *title;
  const char *file_pages;
} list_menu_def;
extern list_menu_def list_menu;

#ifdef __cplusplus
  } /* C-declarations for C++ */
#endif
