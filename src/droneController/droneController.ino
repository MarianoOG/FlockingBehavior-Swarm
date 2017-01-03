// Pins on the board
const int bitPin[8] = {2, 3, 4, 5, 6, 7, 8, 9};

// Variables:
const unsigned short maximum = 170;
byte roll, pitch, yaw, throttle;

void setup() {
// Initialize serial
  Serial.begin(9600);

// Send 0
  for(int i = 0; i<8; i++){
    pinMode(bitPin[i],OUTPUT);
    digitalWrite(bitPin[i],LOW);
  }  
// Start all values as 0
  roll = 0;
  pitch = 0;
  yaw = 0;
  throttle = 0;
   
// Sincronize with quad receptor:
  keyboardPause("Enter to continue...");
  throttle = maximum;
  for(int i = 0; i<8; i++){
    digitalWrite(bitPin[i],bitRead(throttle,i));
  }
  delay(200);
  throttle = 0;
  for(int i = 0; i<8; i++){
    digitalWrite(bitPin[i],bitRead(throttle,i));
  }
  delay(200);
}

void loop() {
// Print current values
  Serial.print("quad = [");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(yaw);
  Serial.print(" | ");
  Serial.print(throttle);
  Serial.print("]\n");
  
// New values:
  throttle = byte(keyboardIntInput("Throttle = "));
  if(throttle>maximum) { throttle = maximum; }
  for(int i = 0; i<8; i++){
    digitalWrite(bitPin[i],bitRead(throttle,i));
  }
  delay(1000);
}

//int keyboardFloatInput(String message) {
//  Serial.println(message);
//  boolean complete = false;
//  int var;
//  while (!complete) {
//    while (Serial.available()) {
//      var = Serial.parseFloat();
//      char inChar = (char)Serial.read();
//      if (inChar == '\n') { complete = true; }
//    }
//  }
//  return var;
//}

int keyboardIntInput(String message) {
  Serial.println(message);
  boolean complete = false;
  int var;
  while (!complete) {
    while (Serial.available()) {
      var = Serial.parseInt();
      char inChar = (char)Serial.read();
      if (inChar == '\n') { complete = true; }
    }
  }
  return var;
}

void keyboardPause(String message) {
  Serial.println(message);
  boolean complete = false;
  while (!complete) {
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == '\n') { complete = true; }
    }
  }
}

