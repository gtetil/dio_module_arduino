  
#include <mcp_can.h>
#include <SPI.h>

//CAN variables
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
MCP_CAN CAN0(10); // Set CS to pin 10

//digital output pins
byte do_0_pin = 8;
byte do_1_pin = 9;
byte do_2_pin = 3;
byte do_3_pin = 4;
byte do_4_pin = 5;
byte do_5_pin = 6;

//digital input pins
byte di_0_pin = 0; //analog input
byte di_1_pin = 1; //analog input
byte di_2_pin = 2; //analog input
byte di_3_pin = 3; //analog input
byte di_4_pin = 4; //analog input
byte di_5_pin = 5; //analog input
byte ignition_pin = 6; //analog input

//card address pins
byte address_bit_0_pin = 7;
byte address_bit_1_pin = 7; //analog input
byte address_bit_2_pin = 0;

//digital output states
byte do_0_state = 0;
byte do_1_state = 0;
byte do_2_state = 0;
byte do_3_state = 0;
byte do_4_state = 0;
byte do_5_state = 0;

//digital input states
byte di_0_state = 0; //analog input
byte di_1_state = 0; //analog input
byte di_2_state = 0; //analog input
byte di_3_state = 0; //analog input
byte di_4_state = 0; //analog input
byte di_5_state = 0; //analog input
byte ignition_state = 0; //analog input

//card address states
byte address_bit_0_state = 0;
byte address_bit_1_state = 0;
byte address_bit_2_state = 0;

//misc
int ai_threshold = 512;
byte card_address = 0;
unsigned long main_timer = millis();
byte data = 0;
byte message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long can_filter = 0x00100000; //nibble 5 is to identify this is for the output message (input from RPi)
unsigned long can_filter_address = 0;

void setup() {
  
  //Serial.begin(115200);  

  //setup digitals
  pinMode(do_0_pin, OUTPUT);
  pinMode(do_1_pin, OUTPUT);
  pinMode(do_2_pin, OUTPUT);
  pinMode(do_3_pin, OUTPUT);
  pinMode(do_4_pin, OUTPUT);
  pinMode(do_5_pin, OUTPUT);

  pinMode(address_bit_0_pin, INPUT_PULLUP);
  pinMode(address_bit_2_pin, INPUT_PULLUP);

  //create card address
  address_bit_0_state = !digitalRead(address_bit_0_pin);
  address_bit_1_state = !readAnalogDI(address_bit_1_pin);
  address_bit_2_state = !digitalRead(address_bit_2_pin);
  bitWrite(card_address, 0, address_bit_0_state);
  bitWrite(card_address, 1, address_bit_1_state);
  bitWrite(card_address, 2, address_bit_2_state);

  can_filter_address = card_address << 16; //shift address into correct spot
  can_filter = can_filter | can_filter_address;

  //CAN setup
  CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  pinMode(2, INPUT); // Setting pin 2 for /INT input

  CAN0.init_Mask(0,0,0x07FF0000); // Init first mask...
  CAN0.init_Filt(0,0,can_filter); // Init first filter...
  CAN0.init_Filt(1,0,can_filter); // Init second filter...
  
  CAN0.init_Mask(1,0,0x07FF0000); // Init second mask... 
  CAN0.init_Filt(2,0,can_filter); // Init third filter...
  CAN0.init_Filt(3,0,can_filter); // Init fouth filter...
  CAN0.init_Filt(4,0,can_filter); // Init fifth filter...
  CAN0.init_Filt(5,0,can_filter); // Init sixth filter...

  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
  
  //setup digitals
  pinMode(do_0_pin, OUTPUT);
  pinMode(do_1_pin, OUTPUT);
  pinMode(do_2_pin, OUTPUT);
  pinMode(do_3_pin, OUTPUT);
  pinMode(do_4_pin, OUTPUT);
  pinMode(do_5_pin, OUTPUT);
  
}

void loop() {

  if ((millis() - main_timer) >= 200) {
    main_timer = millis();
    digitalInputs();
    CAN0.sendMsgBuf(card_address, 0, 8, message);  //id, standard frame, data len, data buf
    //Serial.println(card_address);
    //Serial.println(message[0]);
  }
  
  checkCAN();

  //if ignition is off, then turn off all digital outputs
  if (ignition_state == 0) {
    killAllOutputs();
  }
  
}  

void digitalInputs() {
  di_0_state = readAnalogDI(di_0_pin);
  di_1_state = readAnalogDI(di_1_pin);
  di_2_state = readAnalogDI(di_2_pin);
  di_3_state = readAnalogDI(di_3_pin);
  di_4_state = readAnalogDI(di_4_pin);
  di_5_state = readAnalogDI(di_5_pin);
  ignition_state = readAnalogDI(ignition_pin);

  bitWrite(data, 0, di_0_state);
  bitWrite(data, 1, di_1_state);
  bitWrite(data, 2, di_2_state);
  bitWrite(data, 3, di_3_state);
  bitWrite(data, 4, di_4_state);
  bitWrite(data, 5, di_5_state);
  bitWrite(data, 6, ignition_state);

  message[0] = data;
}

int readAnalogDI(int pin) {
  int aiValue = analogRead(pin);
  byte aiState = 0;
  if (aiValue < ai_threshold) {
    aiState = 1;
  }
  return aiState;
}

void checkCAN() {
  byte output_data = 0;

  if(!digitalRead(2)) { // If pin 2 is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
    output_data = rxBuf[0];

    digitalOutput("DO_0", bitRead(output_data, 0));
    digitalOutput("DO_1", bitRead(output_data, 1));
    digitalOutput("DO_2", bitRead(output_data, 2));
    digitalOutput("DO_3", bitRead(output_data, 3));
    digitalOutput("DO_4", bitRead(output_data, 4));
    digitalOutput("DO_5", bitRead(output_data, 5));
  }
  
}

void digitalOutput(String tag, int state) {
  if (tag == "DO_0") {
    do_0_state = state;
    digitalWrite(do_0_pin, state);
  }
  else if (tag == "DO_1") {
    do_1_state = state;
    digitalWrite(do_1_pin, state);
  }
  else if (tag == "DO_2") {
    do_2_state = state;
    digitalWrite(do_2_pin, state);
  }
  else if (tag == "DO_3") {
    do_3_state = state;
    digitalWrite(do_3_pin, state);
  }
  else if (tag == "DO_4") {
    do_4_state = state;
    digitalWrite(do_4_pin, state);
  }
  else if (tag == "DO_5") {
    do_5_state = state;
    digitalWrite(do_5_pin, state);
  }
}

void killAllOutputs() {
  digitalOutput("DO_0", 0);
  digitalOutput("DO_1", 0);
  digitalOutput("DO_2", 0);
  digitalOutput("DO_3", 0);
  digitalOutput("DO_4", 0);
  digitalOutput("DO_5", 0);
}

