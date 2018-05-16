#include "libs/mavlink/mavlink.h"

unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;
// Launch the serial port in setup
void setup() {
  // MAVLink interface start
  Serial.begin(57600);
}

// Loop your program
void loop() {
  // MAVLink config
  /* The default UART header for your MCU */ 
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  uint8_t system_type = MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
 
  uint8_t system_mode = MAV_MODE_AUTO_ARMED; ///< Booting up
  uint32_t custom_mode = 4;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_POWEROFF; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  mavlink_msg_sys_status_pack(1, 1, &msg, ~0, ~0, ~0, 0, 5,
        3, 2, 0, 0, 0, 0, 0, 0);

    uint16_t message_len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, message_len);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Timing variables
    previousMillisMAVLink = currentMillisMAVLink;

    Serial.write(buf, len);

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Request streams from Pixhawk
      //Mav_Request_Data();
      num_hbs_pasados=0;
    }
  }

  // Check reception buffer
  //comm_receive();
}

void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  // Echo for manual debugging
  // Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
  while(pxSerial.available()>0) {
    uint8_t c = pxSerial.read();
#else
  while(Serial.available()>0) {
    uint8_t c = Serial.read();
#endif

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX HB");
#endif
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("PX SYS STATUS: ");
            mySerial.print("[Bat (V): ");
            mySerial.print(sys_status.voltage_battery);
            mySerial.print("], [Bat (A): ");
            mySerial.print(sys_status.current_battery);
            mySerial.print("], [Comms loss (%): ");
            mySerial.print(sys_status.drop_rate_comm);
            mySerial.println("]");
#endif
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX PARAM_VALUE");
            mySerial.println(param_value.param_value);
            mySerial.println(param_value.param_count);
            mySerial.println(param_value.param_index);
            mySerial.println(param_value.param_id);
            mySerial.println(param_value.param_type);
            mySerial.println("------ Fin -------");
#endif
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
#ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX RAW IMU");
            //mySerial.println(raw_imu.xacc);
#endif
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX ATTITUDE");
            mySerial.println(attitude.roll);
            if(attitude.roll>1) leds_modo = 0;
            else if(attitude.roll<-1) leds_modo = 2;
            else leds_modo=1;
#endif
          }
          break;

        
       default:
#ifdef SOFT_SERIAL_DEBUGGING
          mySerial.print("--- Otros: ");
          mySerial.print("[ID: ");
          mySerial.print(msg.msgid);
          mySerial.print("], [seq: ");
          mySerial.print(msg.seq);
          mySerial.println("]");
#endif
          break;
      }
    }
  }
}
