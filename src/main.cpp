#include <Arduino.h>
#include <CANLib.h>
#include "datatypes.h"
#include "entradas.h"
#include <AS5600.h>
#include "crc.h"

AMS_5600 ams5600;

HardwareSerial Serial2(PA3, PA2);

#define CAN_UPDATE_PERIOD 100 // 10 Hz
#define UPDATE_SENSORS_PERIOD 100
#define BLINK_PERIOD 500
#define DEV_ID 54 // TODO change to UUID hash
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define HW_NAME "OmniDir"
#define UUID "FFFFFFFFFFFF"
#define PAIRING_DONE 0x01
#define FW_TEST_VERSION_NUMBER 0x00
#define FW_NAME ""

// #define FLASHSIZE_BASE        0x1FFFF7E0U    /*!< FLASH Size register base address */
#define STM32_UUID_8 ((uint8_t*)0x1FFFF7E8U) /*!< Unique device ID register base address */
// uint16_t flashsize = *(uint16_t *)(FLASHSIZE_BASE);
// uint32_t UnitID_Part_1 = *(uint32_t*)(UID_BASE );
// uint32_t UnitID_Part_2 = *(uint32_t*)(UID_BASE +0x04);
// uint32_t UnitID_Part_3 = *(uint32_t*)(UID_BASE +0x08);


uint8_t counter = 0;
uint8_t frameLength = 0;
unsigned long previousMillis = 0;     // stores last time output was updated
const long interval = 1000;           // transmission interval (milliseconds)

CAN_msg_t CAN_TX_msg;
CAN_msg_t CAN_RX_msg;



typedef struct
{
  double volts_in;        /* 29 bit identifier                               */
  double  amps_motor;   /* Data field                                      */
  double  volt_can_pos;       /* Length of data field in bytes                   */
  double  celcius_pcb;        /* Object channel(Not use)                         */
  double  dutycycle_motor;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  double  rpm_motor;
  double  pos_motor;
  double  displacement_motor;
} SysState;

SysState sys_state;


int32_t dir, dead_zone_up = 530, dead_zone_down = 530, max_up = 1023, max_down = -1023; // Rango PWM

double convertRawAngleToDegrees(word);

//Variables
void entrada();
float time_0, timer_diff=0, rpm_timer, velocity, jumpsito, premillis, angular_vel;
double pos_travel, pos_sp, pos_diff, first_pos, angle_pos, Previous_Angle;
int i=0, jump;
bool test = false;
uint32_t curr_millis, can_update_millis, update_sensors_millis, blink_millis;
bool flag_can_rx;

void can_send_status_1(CAN_msg_t* msg, SysState* state);
void can_send_status_4(CAN_msg_t* msg, SysState* state);
void can_send_status_5(CAN_msg_t* msg, SysState* state);
void can_send_status_6(CAN_msg_t* msg, SysState* state);
void bldc_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void bldc_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void bldc_buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void bldc_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void bldc_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void bldc_buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
int16_t bldc_buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t bldc_buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t bldc_buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t bldc_buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float bldc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float bldc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
void read_can_data();
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);

void can_send_status_msgs();
void update_sensors();
uint32_t genEId(uint32_t id, uint32_t  packet_id);
void blink();
void process_short_buffer(uint8_t* data);
void send_fw_packets();


void position_correction(double Current_Angle);
void velocity_control(double Current_Angle);
float pos_error, vel_error, vel_cmd, val, posKP=20, velKP=10, velKI=0.03, ki_ev=0;
void canISR();


void setup() 
{
  Serial2.begin(115200);
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
  Serial2.println("Started!");
 
  bool ret = CANInit(CAN_500KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  
  if (!ret)
  {
    Serial2.println("Unable to get CAN"); //TODO Hangs on bootup. Might change this to a restart.
    while(true);
  }

  CANattachInterrupt(canISR);
  CANfilterMask32Init(0, DEV_ID, 0x000000FF);
}

void loop() 
{
  curr_millis = millis();

  if(flag_can_rx)
  {
    read_can_data();
    flag_can_rx = false;
  }

  if (curr_millis > can_update_millis)
  {
    can_update_millis = curr_millis + CAN_UPDATE_PERIOD;
    can_send_status_msgs();
  }

  if (curr_millis > update_sensors_millis)
  {
    update_sensors_millis = curr_millis + UPDATE_SENSORS_PERIOD;
    update_sensors();
  }

  // if (curr_millis > read_encoder_millis)
  // {
  //   read_encoder_millis = curr_millis + READ_ENCODER_PERIOD;
  //   read_encoder();
  // }


  if (curr_millis > blink_millis)
  {
    blink_millis = curr_millis + BLINK_PERIOD;
    blink();
  }


  // CAN_TX_msg.data[0] = 0x00;
  // CAN_TX_msg.data[1] = 0x01;
  // CAN_TX_msg.data[2] = 0x02;
  // CAN_TX_msg.data[3] = 0x03;
  // CAN_TX_msg.data[4] = 0x04;
  // CAN_TX_msg.data[5] = 0x05;
  // CAN_TX_msg.data[6] = 0x06;
  // CAN_TX_msg.data[7] = 0x07;
  // CAN_TX_msg.len = frameLength;

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   digitalWrite(PC13, LOW);
  //   previousMillis = currentMillis;
  //   if ( ( counter % 2) == 0) {
  //     CAN_TX_msg.type = DATA_FRAME;
  //     if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
  //     CAN_TX_msg.format = EXTENDED_FORMAT;
  //     CAN_TX_msg.id = 0x32F103;
  //   } else {
  //     CAN_TX_msg.type = DATA_FRAME;
  //     if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
  //     CAN_TX_msg.format = STANDARD_FORMAT;
  //     CAN_TX_msg.id = 0x103;
  //   }
  //   CANSend(&CAN_TX_msg);
  //   frameLength++;
  //   if (frameLength == 9) frameLength = 0;
  //   counter++;
  // }
  
  // if(CANMsgAvail()) {
  //   CANReceive(&CAN_RX_msg);

  //   if (CAN_RX_msg.format == EXTENDED_FORMAT) {
  //     Serial.print("Extended ID: 0x");
  //     if (CAN_RX_msg.id < 0x10000000) Serial.print("0");
  //     if (CAN_RX_msg.id < 0x1000000) Serial.print("00");
  //     if (CAN_RX_msg.id < 0x100000) Serial.print("000");
  //     if (CAN_RX_msg.id < 0x10000) Serial.print("0000");
  //     Serial.print(CAN_RX_msg.id, HEX);
  //   } else {
  //     Serial.print("Standard ID: 0x");
  //     if (CAN_RX_msg.id < 0x100) Serial.print("0");
  //     if (CAN_RX_msg.id < 0x10) Serial.print("00");
  //     Serial.print(CAN_RX_msg.id, HEX);
  //     Serial.print("     ");
  //   }

  //   Serial.print(" DLC: ");
  //   Serial.print(CAN_RX_msg.len);
  //   if (CAN_RX_msg.type == DATA_FRAME) {
  //     Serial.print(" Data: ");
  //     for(int i=0; i<CAN_RX_msg.len; i++) {
  //       Serial.print("0x"); 
  //       Serial.print(CAN_RX_msg.data[i], HEX); 
  //       if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
  //     }
  //     Serial.println();
  //   } else {
  //     Serial.println(" Data: REMOTE REQUEST FRAME");
  //   }
  // }
  

}

void blink()
{
  static bool state;
  state = !state;
  digitalWrite(PC13, state);
}

void canISR() // get CAN bus frame passed by a filter into fifo0
{
  // Serial2.println("GOT can packet!");
  CANReceive(&CAN_RX_msg);
  flag_can_rx = true;
}

void read_can_data()
{
  Serial2.println("Got CAN packet!");
  uint32_t packet_type = (0xFF00 & CAN_RX_msg.id)>>8;
  
  switch (packet_type)
  {
    case CAN_PACKET_PROCESS_SHORT_BUFFER:
      Serial2.println("Got CAN_PACKET_PROCESS_SHORT_BUFFER");
      process_short_buffer(CAN_RX_msg.data);
      break;
  }

}

void process_short_buffer(uint8_t* data)
{
  switch (data[1])
  {
    case COMM_FW_VERSION:
      Serial2.println("Got COMM_FW_VERSION");
      send_fw_packets();
  }
}

void send_fw_packets()
{
  // char fw_buff[100];
  // int32_t fw_len = snprintf(fw_buff, sizeof(fw_buff), "%x%x%x%s%s%x%x%x%x", COMM_FW_VERSION, FW_VERSION_MAJOR, FW_VERSION_MINOR, "OmniDir\n", UUID, PAIRING_DONE, TEST_VERSION, 0,0);
  // if (fw_len < 0) return;

  // Serial2.println(fw_buff);

  int32_t ind = 0;
  uint8_t send_buffer[65];
  send_buffer[ind++] = COMM_FW_VERSION;
  send_buffer[ind++] = FW_VERSION_MAJOR;
  send_buffer[ind++] = FW_VERSION_MINOR;

  strcpy((char*)(send_buffer + ind), HW_NAME);
  ind += strlen(HW_NAME) + 1;

  memcpy(send_buffer + ind, STM32_UUID_8, 12);
  ind += 12;

  // // Add 1 to the UUID for the second motor, so that configuration backup and
  // // restore works.
  // if (mc_interface_get_motor_thread() == 2) {
  //   send_buffer[ind - 1]++;
  // }

  // send_buffer[ind++] = app_get_configuration()->pairing_done;
  send_buffer[ind++] = PAIRING_DONE;

  send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

  // send_buffer[ind++] = HW_TYPE_VESC;
  send_buffer[ind++] = 0;


  send_buffer[ind++] = 0; // No custom config

#ifdef HW_HAS_PHASE_FILTERS
		send_buffer[ind++] = 1;
#else
		send_buffer[ind++] = 0;
#endif

#ifdef QMLUI_SOURCE_HW
#ifdef QMLUI_HW_FULLSCREEN
		send_buffer[ind++] = 2;
#else
		send_buffer[ind++] = 1;
#endif
#else
		send_buffer[ind++] = 0;
#endif

	  send_buffer[ind++] = 0;
		send_buffer[ind++] = 0;

		strcpy((char*)(send_buffer + ind), FW_NAME);
		ind += strlen(FW_NAME) + 1;

		// fw_version_sent_cnt++;

		// reply_func(send_buffer, ind);
    comm_can_send_buffer(DEV_ID, send_buffer, ind, 1);

  
	
}

void can_send_status_1(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int32(buff,(int32_t)state->rpm_motor, &i);
  bldc_buffer_append_int16(buff,(int16_t)state->amps_motor * 1e3, &i);
  bldc_buffer_append_int16(buff,(int16_t)state->dutycycle_motor * 1e3, &i);
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);

  CANSend(msg);
}

void can_send_status_4(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int16(buff,(int32_t)(state->celcius_pcb * 1e1), &i);
  bldc_buffer_append_int16(buff,23 * 1e1, &i); //TEMP MOTOR
  bldc_buffer_append_int16(buff,(int16_t)(state->amps_motor * 1e1), &i);
  bldc_buffer_append_int16(buff,(int16_t)(state->pos_motor * 50.0), &i);
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS_4);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);

  CANSend(msg);
}

void can_send_status_5(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int32(buff,(int32_t)state->displacement_motor, &i);
  bldc_buffer_append_int16(buff,(int32_t)(state->volts_in*1e1), &i);
  bldc_buffer_append_int16(buff,0, &i);
  
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS_5);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);
  CANSend(msg);
}
void can_send_status_6(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int16(buff,0 * 1e1, &i);
  bldc_buffer_append_int16(buff,(int16_t)(state->volt_can_pos*1e3), &i);
  bldc_buffer_append_int16(buff,0.0, &i);
  bldc_buffer_append_int16(buff,0.0, &i);
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS_6);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);
  for (int i = 0; i < 8; i++)
  {
    Serial2.print(buff[i],HEX);
    Serial2.print(" ");
  }
  Serial2.println();
  CANSend(msg);
}

void can_send_status_msgs()
{
  // Serial2.println("Sending Status Messages");
  can_send_status_1(&CAN_TX_msg, &sys_state);
  can_send_status_4(&CAN_TX_msg, &sys_state);
  can_send_status_5(&CAN_TX_msg, &sys_state);
  can_send_status_6(&CAN_TX_msg, &sys_state);
}

void entrada()
{
  if (!test)
  {
    time_0 = millis();
    rpm_timer = millis();
    premillis = millis();
    test = true;
  }

  jump = (millis()-rpm_timer);
  timer_diff = (millis()-time_0);
  if (timer_diff > 0 && timer_diff <= 1322) //Tiempo que dura el moviemiento
  {
    if (jump >= 20)
    {
      if (first_pos > 270)
      {
        velocity = entradas[0][i+1]*-1;
        pos_travel -= entradas[1][i+1];
        pos_sp = pos_travel + first_pos;
      }
      else
      {
        velocity = entradas[0][i+1];
        pos_travel += entradas[1][i+1];
        pos_sp = pos_travel + first_pos;
      }
      i = i+1;
      rpm_timer = millis();
      Serial2.print(timer_diff/1000, 3);
      // Serial2.print(" pos_sp: ");
      // Serial2.print(pos_sp);
      Serial2.print(" angle: ");
      Serial2.print(angle_pos);
      // Serial2.print(" | vel_sp: ");
      // Serial2.print(velocity);
      Serial2.print(" rpm: ");
      Serial2.print(angular_vel);
      Serial2.print(" ");
      Serial2.println(val);
    }
    angle_pos = convertRawAngleToDegrees(ams5600.getRawAngle());
    position_correction(angle_pos);
  }
  else if (timer_diff > 1322 && timer_diff < 1800)
  {
    analogWrite(PB8, 1);
    analogWrite(PB9, 1);
    angle_pos = convertRawAngleToDegrees(ams5600.getRawAngle());
    Serial2.println(angle_pos);
  }
  else if (timer_diff > 1800)
  {
    analogWrite(PB8, 1);
    analogWrite(PB9, 1);
  }
}

void position_correction(double Current_Angle)
{
  pos_error = pos_sp - Current_Angle; //// Position error
  vel_cmd = pos_error*posKP+velocity;
  velocity_control(Current_Angle);
}

void velocity_control(double Current_Angle)
{
  jumpsito = millis()-premillis;
  if (jumpsito >= 2)
  {
    pos_diff = Current_Angle - Previous_Angle;
    premillis = millis();
    Previous_Angle = Current_Angle;
    angular_vel = (pos_diff/(jumpsito))*1000/6; ///// VELOCITY FEEDBACK
    vel_error = vel_cmd - angular_vel; ///// Velocity error
    ki_ev = vel_error*velKI+ki_ev;
    val = vel_error*velKP + ki_ev; ///// Valor PWM

    if (val  > 0)
    {
      val = val + dead_zone_up;
      if(val > max_up)
      {
        val = max_up;
        ki_ev = 0;
      }

      analogWrite(PB9, 0);
      analogWrite(PB8, val);
    }
    else
    {
      val = val-dead_zone_down;
      if(val < max_down)
      {
        val = max_down;
        ki_ev = 0;
      }
      val = val*-1;
      analogWrite(PB8, 0);
      analogWrite(PB9, val);
    }    
  }
}

void update_sensors()
{
  sys_state.celcius_pcb = (3.3*((double)analogRead(PA0)/1023)); //TODO change to celsius
  sys_state.volt_can_pos = (3.3*((double)analogRead(PA1)/1023));
  sys_state.volts_in = (3.3*5.5454*((double)analogRead(PA4)/1023));
  sys_state.amps_motor = ((3.3*((double)analogRead(PA5)/1023))/0.2);
  Serial2.println(sys_state.celcius_pcb);
  Serial2.println(sys_state.volt_can_pos);
  Serial2.println(sys_state.volts_in);
  Serial2.println(sys_state.amps_motor);
}

double convertRawAngleToDegrees(word newAngle)
{
  if (newAngle == 0xFFFF) return 0xFFFF;
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}

void bldc_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
  bldc_buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void bldc_buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
  bldc_buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

int16_t bldc_buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res =   ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

uint16_t bldc_buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
  uint16_t res =  ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

int32_t bldc_buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res =   ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

uint32_t bldc_buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
  uint32_t res =  ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

float bldc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)bldc_buffer_get_int16(buffer, index) / scale;
}

float bldc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)bldc_buffer_get_int32(buffer, index) / scale;
}


uint32_t genEId(uint32_t id, uint32_t  packet_id)
{
  id |= packet_id << 8;  // Next lowest byte is the packet id.
  return(id |= 0x80000000);              // Send in Extended Frame Format.
}

void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) 
{
	uint8_t send_buffer[8];

	if (len <= 6) 
  {
		uint32_t ind = 0;
		send_buffer[ind++] = DEV_ID;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
    CAN_TX_msg.id = genEId(DEV_ID, CAN_PACKET_PROCESS_SHORT_BUFFER);
    memcpy(CAN_TX_msg.data, send_buffer, ind);
    CAN_TX_msg.format = EXTENDED_FORMAT;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.len = len;
		CANSend(&CAN_TX_msg);
	} 
  else 
  {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) 
    {
			if (i > 255) 
      {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) 
      {
				memcpy(send_buffer + 1, data + i, send_len);
			} 
      else 
      {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

      CAN_TX_msg.id = genEId(DEV_ID, CAN_PACKET_FILL_RX_BUFFER);
      memcpy(CAN_TX_msg.data, send_buffer, send_len+1);
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.type = DATA_FRAME;
      CAN_TX_msg.len = send_len+1;
      CANSend(&CAN_TX_msg);

			// comm_can_transmit_eid_replace(controller_id |
			// 		((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1, true, 0);
		}

		for (unsigned int i = end_a;i < len;i += 6) 
    {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

      CAN_TX_msg.id = genEId(DEV_ID, CAN_PACKET_FILL_RX_BUFFER_LONG);
      memcpy(CAN_TX_msg.data, send_buffer, send_len+2);
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.type = DATA_FRAME;
      CAN_TX_msg.len = send_len+2;
      CANSend(&CAN_TX_msg);

			// comm_can_transmit_eid_replace(controller_id |
			// 		((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2, true, 0);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = DEV_ID;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

    CAN_TX_msg.id = genEId(DEV_ID, CAN_PACKET_PROCESS_RX_BUFFER);
    memcpy(CAN_TX_msg.data, send_buffer, ind);
    CAN_TX_msg.format = EXTENDED_FORMAT;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.len = ind++;
    CANSend(&CAN_TX_msg);

		// comm_can_transmit_eid_replace(controller_id |
		// 		((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++, true, 0);
	}
}