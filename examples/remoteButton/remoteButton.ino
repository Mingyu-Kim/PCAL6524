#include <Wire.h>
#include "PCAL6254.h"


/* This code is desinged to mimic the Button schetch found in the basic arduino library sample code 
 * but with using the IO on the PCAL6254 insted of the built in IO
 * 
 * Please place a LED between the deisred pin and ground.
 * 
 * In this schetch the Input Pin is Set to P0_1 and the Output Pin is set to P0_0
 */



PCAL6524 io;  //Address Pin is tied to ground (DEFAULT)
//PCAL6524 io(PCAL6524_ADDRESS_0); //Adress Pin is Tied to SCL
//PCAL6524 io(PCAL6524_ADDRESS_1); //Adress Pin is Tied to SDA
//PCAL6524 io(PCAL6524_ADDRESS_3); //Adress Pin is Tied to VDD

/* PCAL6524 24 IO pins are brocken into 3 Banks, Pin IDs coresponed to Pin IDs from NXP data sheets
 *    Bank 0       Bank 1     Bank 2
 *    P0[0]         P1[1]       P2[1]
 *    P0[1]         P1[1]       P2[1]
 *    P0[2]         P1[2]       P2[2]
 *    P0[3]         P1[3]       P2[3]
 *    P0[4]         P1[4]       P2[4]    
 *    P0[5]         P1[5]       P2[5]
 *    P0[6]         P1[6]       P2[6]
 *    P0[7]         P1[7]       P2[7]
 */
// variables will change:
int buttonState = 0;         // variable for reading the pushbutton statu



void setup() {
  
  //Join the MCU to the I2C buss as its master and resest the PCAL6524 to default;
  io.begin();
  //Initialize the desired Pin to desired mode (Input, or Output)
  io.pinMode(P0[0], OUTPUT); //Seting Pin P0_0 to an Output;
  io.pinMode(P0[1], INPUT);  //Sets Pin P0_1 to be an Input;

}
void loop() {
  // read the state of the pushbutton value:
  buttonState = io.digitalRead(P0[1]);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH)
  {
    //turn LED ON
    io.digitalWrite(P0[0], HIGH);
  }
  else{  
  //turn off ON
  io.digitalWrite(P0[0], LOW);
  }
}
