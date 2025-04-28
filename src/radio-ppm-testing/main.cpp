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
  Serial.print(" 4: ");
  Serial.print(ReceiverValue[4]);
  Serial.print(" 5: "); 
  Serial.print(ReceiverValue[5]);
  Serial.print(" 6: "); 
  Serial.print(ReceiverValue[6]);
  Serial.print(" 7: "); 
  Serial.println(ReceiverValue[7]);
  delay(50);
}

void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i=1; i<=ChannelNumber;i++){ // index starts at 0 but receiver input at 1
      ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}