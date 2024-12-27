#include <Arduino.h>
#include <PulsePosition.h>

#define HWSERIAL Serial1 // Set to one of the Teensy's hardware serial 

PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0; 

// put function declarations here:
void read_receiver();


void setup(){
  Serial.begin(9600);
  HWSERIAL.begin(57600); // 57600 seems locked in from the tleemtry modules themselves, need to try higher
  ReceiverInput.begin(6); // PPM capable pin on Teensy 4.1
}

void loop(){
  read_receiver();
  Serial.print("Number of channels: ");
  Serial.print(ChannelNumber);
  Serial.print(" Roll [µs]: ");
  Serial.print(ReceiverValue[0]);
  Serial.print(" Pitch [µs]: "); 
  Serial.print(ReceiverValue[1]);
  Serial.print(" Throttle [µs]: "); 
  Serial.print(ReceiverValue[2]);
  Serial.print(" Yaw [µs]: "); 
  Serial.println(ReceiverValue[3]);
  delay(50);
  HWSERIAL.print("Number of channels: ");
  HWSERIAL.print(ChannelNumber);
  HWSERIAL.print(" Roll [µs]: ");
  HWSERIAL.print(ReceiverValue[0]);
  HWSERIAL.print(" Pitch [µs]: "); 
  HWSERIAL.print(ReceiverValue[1]);
  HWSERIAL.print(" Throttle [µs]: "); 
  HWSERIAL.print(ReceiverValue[2]);
  HWSERIAL.print(" Yaw [µs]: "); 
  HWSERIAL.println(ReceiverValue[3]);
}

void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i=1; i<=ChannelNumber;i++){ // index starts at 0 but receiver input at 1
      ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}