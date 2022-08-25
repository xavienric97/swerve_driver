#include <Arduino.h>
#include <CANLib.h>
#include "datatypes.h"
#include "entradas.h"
#include <AS5600.h>

AMS_5600 ams5600;

#define CAN_UPDATE_PERIOD 100 // 10 Hz
#define UPDATE_SENSORS_PERIOD 100
#define DEV_ID 54 // TODO change to UUID hash

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
uint32_t curr_millis, can_update_millis, update_sensors_millis;

void gen_can_status_1(CAN_msg_t* msg, SysState* state);
void gen_can_status_4(CAN_msg_t* msg, SysState* state);
void gen_can_status_5(CAN_msg_t* msg, SysState* state);
void gen_can_status_6(CAN_msg_t* msg, SysState* state);
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

void can_send_status_msgs();
void update_sensors();
uint32_t genEId(uint32_t id, uint32_t  packet_id);

void position_correction(double Current_Angle);
void velocity_control(double Current_Angle);
float pos_error, vel_error, vel_cmd, val, posKP=20, velKP=10, velKI=0.03, ki_ev=0;


void setup() 
{
  Serial.begin(115200);
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
  Serial.println("Started!");
 
  bool ret = CANInit(CAN_500KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  
  if (!ret)
  {
    Serial.println("Unable to get CAN"); //TODO Hangs on bootup. Might change this to a restart.
    while(true);
  }
}

void loop() 
{
  curr_millis = millis();

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

void gen_can_status_1(CAN_msg_t* msg, SysState* state)
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

void gen_can_status_4(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int16(buff,(int32_t)state->celcius_pcb * 1e1, &i);
  bldc_buffer_append_int16(buff,23 * 1e1, &i);
  bldc_buffer_append_int16(buff,(int16_t)state->amps_motor * 1e1, &i);
  bldc_buffer_append_int16(buff,(int16_t)state->pos_motor * 50.0, &i);
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS_4);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);
  CANSend(msg);
}

void gen_can_status_5(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int32(buff,(int32_t)state->displacement_motor, &i);
  bldc_buffer_append_int32(buff,(int32_t)state->volts_in, &i);
  
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS_5);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);
  CANSend(msg);
}
void gen_can_status_6(CAN_msg_t* msg, SysState* state)
{
  uint8_t buff[8];
  int32_t i = 0;
  bldc_buffer_append_int16(buff,0 * 1e1, &i);
  bldc_buffer_append_int16(buff,(int16_t)state->pos_motor, &i);
  bldc_buffer_append_int16(buff,0.0, &i);
  bldc_buffer_append_int16(buff,0.0, &i);
  memcpy(msg->data, buff, 8);
  msg->id = genEId(DEV_ID, CAN_PACKET_STATUS_6);
  msg->format = EXTENDED_FORMAT;
  msg->type = DATA_FRAME;
  msg->len = sizeof(buff);
  CANSend(msg);
}

void can_send_status_msgs()
{
  gen_can_status_1(&CAN_TX_msg, &sys_state);
  gen_can_status_4(&CAN_TX_msg, &sys_state);
  gen_can_status_5(&CAN_TX_msg, &sys_state);
  gen_can_status_6(&CAN_TX_msg, &sys_state);
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
