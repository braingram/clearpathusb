#include "USBHost_t36.h"
uint32_t format = USBHOST_SERIAL_8N1;


USBHost myusb;
//USBHIDParser hid1(myusb);
USBHub hub1(myusb);
USBHub hub2(myusb);
USBSerial userial(myusb);
bool userial_active = false;

// a: write high to move
#define A_PIN 2
// en: write high to enable
#define EN_PIN 3
// fb pin: setup as ASG, goes low when done moving
#define FB_PIN 4

float velocity_scalar = 8279.2205;
float acceleration_scalar = 2.45138;

byte msg_buffer[16];


byte compute_checksum(byte* msg, byte len) {
  byte sum = 0xFF;
  for (int i=0; i<len; i++) {
    sum -= msg[i];
  }
  return (sum + 1) & 0x7F;
}

int send_velocity(float vel) {
  if ((vel < 0x01) || (vel > 4000)) return 1;
  msg_buffer[0] = 128;
  msg_buffer[1] = 7;
  msg_buffer[2] = 1;
  msg_buffer[3] = 122;
  int sv = (int)(vel * velocity_scalar);
  msg_buffer[4] = ((sv & 0x1F) << 2);
  msg_buffer[5] = ((sv >> 5) & 0x7F);
  msg_buffer[6] = ((sv >> 12) & 0x7F);
  msg_buffer[7] = ((sv >> 19) & 0x7F);
  msg_buffer[8] = 0x00;
  msg_buffer[9] = compute_checksum(msg_buffer, 9);
  // send
  userial.write(msg_buffer, 10);
  /*
  for (int i=0; i< 10; i++) {
    Serial.print(msg_buffer[i], HEX);
    Serial.print(',');
  }
  Serial.println();
  */
  msg_buffer[4] |= 0x2;
  msg_buffer[9] = compute_checksum(msg_buffer, 9);
  // send
  userial.write(msg_buffer, 10);
  /*
  for (int i=0; i< 10; i++) {
    Serial.print(msg_buffer[i], HEX);
    Serial.print(',');
  }
  Serial.println();
  */
  return 0;
}

int send_acceleration(float acc) {
  if ((acc < 0x01) || (acc > 4000)) return 1;
  msg_buffer[0] = 128;
  msg_buffer[1] = 7;
  msg_buffer[2] = 1;
  msg_buffer[3] = 124;
  int sv = (int)(acc * acceleration_scalar);
  msg_buffer[4] = ((sv & 0x1F) << 2);
  msg_buffer[5] = ((sv >> 5) & 0x7F);
  msg_buffer[6] = ((sv >> 12) & 0x7F);
  msg_buffer[7] = ((sv >> 19) & 0x7F);
  msg_buffer[8] = 0x00;
  msg_buffer[9] = compute_checksum(msg_buffer, 9);
  // send
  userial.write(msg_buffer, 10);
  msg_buffer[4] |= 0x2;
  msg_buffer[9] = compute_checksum(msg_buffer, 9);
  // send
  userial.write(msg_buffer, 10);
  return 0;
}

byte set_baud_message[] = {0xF0, 0x03, 0x09, 0x30, 0x00, 0x54};

int fb_val = 0;
elapsedMillis t;
elapsedMillis pt;

void setup() {
  // put your setup code here, to run once:
  pinMode(FB_PIN, INPUT_PULLUP);
  pinMode(A_PIN, OUTPUT);
  digitalWrite(A_PIN, LOW);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(9600);
  fb_val = digitalRead(FB_PIN);
  Serial.print("FB: "); Serial.println(fb_val);

  myusb.begin();
}

void loop() {
  myusb.Task();
  /*
  if (t < 3000) {
    myusb.Task();
  } else {
    return;
  }
  if (pt >= 100) {
    Serial.println("Done checking usb...");
    pt = 0;
  };
  return;
  */
  if (userial != userial_active) {
    if (userial_active) {
      // disconnect
      Serial.println("USB Serial disconnected");
      userial_active = false;
    } else {
      // connect
      Serial.println("USB Serial connected");
      userial_active = true;
      Serial.println("Setting initial baud rate");
      userial.begin(9600);
      delay(500);
      Serial.println("Sending baud rate change");
      userial.write(set_baud_message, sizeof(set_baud_message));
      delay(500);
      Serial.print("Read:[");
      while (userial.available()) {
        Serial.print(userial.read(), HEX);
        Serial.print(",");
      }
      Serial.println("]");
      userial.end();
      Serial.println("Setting final baud rate");
      userial.begin(230400);
    }
  }

  // TODO read userial?
  // put your main code here, to run repeatedly:
  float fv = 0.0;
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'e':
        digitalWrite(EN_PIN, !digitalRead(EN_PIN));
        break;
      case 'a':
        digitalWrite(A_PIN, !digitalRead(A_PIN));
        break;
      case 'u':
        Serial.print("Vendor: "); Serial.println(userial.idVendor(), HEX);
        Serial.print("Product: "); Serial.println(userial.idProduct(), HEX);
        break;
      case 'V':
        fv = Serial.parseFloat();
        Serial.print("Setting velocity to: ");
        Serial.print(fv);
        Serial.print(" returned ");
        Serial.println(send_velocity(fv));
        break;
      case 'A':
        fv = Serial.parseFloat();
        Serial.print("Setting acceleration to: ");
        Serial.print(fv);
        Serial.print(" returned ");
        send_acceleration(fv);
        break;
    }
  }
  if (digitalRead(FB_PIN) != fb_val) {
    fb_val = 1 - fb_val;
    Serial.print("FB: "); Serial.println(fb_val);
  }
}
