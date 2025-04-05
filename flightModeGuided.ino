#include <Arduino.h>
#include "MAVLink.h"

static const uint8_t ARDUINO_SYS_ID  = 42;   // Arduino's system ID
static const uint8_t ARDUINO_COMP_ID = 200;  // Arduino's component ID
static const uint8_t FC_SYS_ID       = 1;    // flight controller's system ID (usually 1)
static const uint8_t FC_COMP_ID      = 1;    // flight controller's component ID (usually 1)

// Limit switch wiring:
// C (common) connected to GPIO pin
// NO (normally open ) or NC (normally closed) connected to ground
//IMPORTANT: For arduino nano esp32, use pins listed on board itself NOT corresponding gpio pins on pinout chart!!
static const uint8_t switchPin = 7; // pin connected to arduino for limit switch
bool currentState;
bool prevState;

// RX and TX pins for mavlink comm with flight controller
// remember rx->tx and tx->rx
// Set corresponding serial in ardupilot to 2 (mavlink 2) and baud rate to 57600
//IMPORTANT: For arduino nano esp32, use pins listed on board itself NOT corresponding gpio pins on pinout chart!!
static const int rxPin = 4;
static const int txPin = 5;

void setup() {
  Serial.begin(115200); // usb c serial for testing while connected to computer
  delay(300);

  Serial2.begin(57600, SERIAL_8N1, rxPin, txPin); // serial for  mavlink
  Serial2.setPins(rxPin, txPin);
  delay(300);

  pinMode(switchPin, INPUT_PULLUP); // for limit switch
  currentState = digitalRead(switchPin) == LOW ? true : false;

  Serial.println("Serials started");
  delay(10000);
  armForce();
  Serial.println("init arm");


}

void loop() {
  // if (Serial2.available() > 0) printNextNHeartbeats(1);
  prevState = currentState;
  currentState = digitalRead(switchPin) == LOW ? true : false;

  if (currentState != prevState){
    if (currentState == true){
      Serial.println("Disarm command");
      armForce();
      delay(50);
      setFlightMode(0); // MANUAL / no input = 0
    }
    else{
      Serial.println("Arm command");
      armForce();
      delay(50);
      setFlightMode(15); // Guided for scripting = 15, AUTO = 10
    }
  }

  delay(10);
}

void armForce() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
      ARDUINO_SYS_ID, 
      ARDUINO_COMP_ID,
      &msg,
      FC_SYS_ID,                 // target system (your flight controller)
      FC_COMP_ID,                // target component (commonly 1)
      MAV_CMD_COMPONENT_ARM_DISARM, 
      0,                         // confirmation
      1.0f,                      // param1 = 1 => arm
      21196.0f,                  // param2 = 21196 => force
      0, 0, 0, 0, 0             // param3..7 unused
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void disarmForce() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    ARDUINO_SYS_ID, 
    ARDUINO_COMP_ID,
    &msg,
    FC_SYS_ID,                // target system
    FC_COMP_ID,               // target component
    MAV_CMD_COMPONENT_ARM_DISARM, 
    0,                        // confirmation
    0.0f,                     // param1 = 0 => disarm
    21196.0f,                 // param2 = 21196 => force
    0, 0, 0, 0, 0            // param3..7 unused
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);  // or Serial2, etc., depending on your hardware
}

void setFlightMode(uint32_t mode) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Use MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1<<7 = 128 decimal)
  uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; 

  // Pack the message
  mavlink_msg_set_mode_pack(
      ARDUINO_SYS_ID, 
      ARDUINO_COMP_ID, 
      &msg, 
      FC_SYS_ID, 
      base_mode, 
      mode
  );

  // Serialize the message
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void printNextNHeartbeats(int count) {
  int receivedCount = 0;

  // We loop until we've seen 'count' heartbeat messages
  while (receivedCount < count) {
    // Parse incoming bytes
    if (Serial2.available() > 0) {
      uint8_t c = Serial2.read();

      static mavlink_message_t msg;
      static mavlink_status_t status;

      // Feed the byte into MAVLink parser
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        // We got a full MAVLink message
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          // Increase the count
          receivedCount++;

          Serial.print("Update #");
          Serial.print(receivedCount);
          Serial.println(":");
          Serial.println("0 = stabilize, 3 = auto");

          // The flight mode is in 'hb.custom_mode'
          Serial.print("  custom_mode = ");
          Serial.println(hb.custom_mode);

          bool isArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

          Serial.print("Status: ");
          Serial.println(isArmed ? "Armed" : "Disarmed");



          // // For convenience, also print base_mode, system ID, etc.
          // Serial.print("  base_mode   = ");
          // Serial.println(hb.base_mode);
          // Serial.print("  sysid       = ");
          // Serial.println(msg.sysid);
          // Serial.print("  compid      = ");
          // Serial.println(msg.compid);

          // Serial.println("-------------------------");
        }
      }
    }
    // Optionally add a small delay to avoid busy looping
    delay(1);
  }

  // Serial.println("Finished reading heartbeat messages.");
}
