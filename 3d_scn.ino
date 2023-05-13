#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>
#include <SharpIR.h>
#include <Stepper.h>

//Editable variables
//int Avg_distance = 40;                  //Amount of scans for each point. The result is the mean. This would increase the delay for each scan.
String file="scan_001.txt";             
//Name of the saved file on the SD card
int z_axis_height = 12; //in cm         //Maximum height of the scaned file
float z_layer_height = 0.5; //in mm     //Layer height. The amount of mm for each layer. 
int lead_screw_rotations_per_cm = 8;    //How many rotations needs the lead screw to make in order to make 1cm.
int Turntable_motor_steps = 200;        //Steps that the motor needs for a full rotation. 
int distance_to_center = 14;            //In cm. Distance from sensor to the turntable center in cm

int pinA = 9, pinAA = 10, pinB = 7, pinBB = 8; 
AccelStepper mystepper2(AccelStepper::HALF4WIRE, pinA, pinAA, pinB, pinBB);
int pinA1 = 2, pinA2 = 3, pinB1 = 4, pinB2 = 5; 
AccelStepper mystepper(4, pinA1, pinA2, pinB1, pinB2);

// //I/O
int csPin = 6;
#define IRPin A0
#define model 1080

SharpIR mySensor = SharpIR(IRPin, model);

//Variables
File file_values;           //Used for the SD card module
int scan = 0;               //Activate/deactivate scanning
int scan_changed = 0;       //Scan process was changed
float distance_cm;          //Measured distance
float angle = 0;            //Rotation angle for each loop (0º-360º)
float x = 0;                //X, Y and Z coordinate
float y = 0;
float z = 0;
int z_loop = 0;             //variable used for the z-axis motor rotation
int r_loop = 0;             //variable used for the turntable motor rotation
float RADIANS = 0.0;        //Angle in radians. We calculate this value later in Setup loop
int steps_z_height = 0;     //Variable used for the amount of steps in z-axis


void setup() {
  Serial.begin(9600);
  mystepper.setMaxSpeed(900);
  mystepper2.setMaxSpeed(900);
	mystepper.setSpeed(500);
  mystepper2.setSpeed(200);
  SD.begin(csPin);

  //Calculate variables
  RADIANS = (3.141592 / 180.0) * (360/Turntable_motor_steps);
  steps_z_height = (z_layer_height * Turntable_motor_steps * lead_screw_rotations_per_cm)/10;

}

void loop(){
  //mystepper2.runSpeed();
// {
// //Wait till the push button is pressed
// if(digitalRead(button))
// {
//   if(scan == 1 && scan_changed == 0)
//   {
//     scan=0;
//     delay(3000);
//     scan_changed=1;
//   }
//   if(scan == 0 && scan_changed == 0)
//   {
//     scan=1;
//     delay(3000);
//     scan_changed=1;
//   }
//   scan_changed = 0;   
// }


// //If the scanning proces is ON
// if(scan == 1)
// { 
  //We stop when we reach the maximum height
  if(z < z_axis_height)
  {     
    for(int loop_cont = 0 ; loop_cont < Turntable_motor_steps; loop_cont++)
    {
      getDistance();
      mystepper.runSpeed();
      angle = angle + RADIANS;      //Increase the angle by one more unit
      write_to_SD(x,y,z);           //Write x, y, z files to SD card function
   
      //Serial.print(loop_cont);     Serial.print("   "); 
      //Serial.print(angle);     Serial.print("   "); 
      Serial.println(distance_cm);     Serial.print("   "); 
      //Serial.print(x);     Serial.println("   "); 
      //Serial.print(y);     Serial.print("   "); 
      //Serial.print(z);     Serial.println("   "); 
    }
    angle = 0;  //Reset the angle value for next rotation
    
    while(z_loop < steps_z_height) 
    {
      Serial.println("clockwise2");
      mystepper2.runSpeed();
      delay(1);
      z_loop = z_loop + 1;              //Increase the loop by 1
    }
    z = z + z_layer_height;         //Increase the made z-height by 1 unit
    z_loop = 0;                     //Reset the z-axis rotation variable
    }                               //end of if z_height 
//}
}


//Function that gets the distance from sensor
double getDistance(){
// {
//   for (int i=0; i < Avg_distance; i++)
//   {
//     measured_analog = analogRead(A0);
//     delay(2);
//     analog = analog + measured_analog;    
//   }
//   distance_cm = analog/Avg_distance;        //Get the mean. Divide the scan by the amount of scans. 
//   analog=0;  //reset the andlog read total value
//   measured_analog = 0; //reset the andlog read value
//   distance_cm = mapFloat(distance_cm,0.0,1023.0,0.0,5.0); //Convert analog pin reading to voltage
//   distance_cm = 1/((distance_cm-1125)/137500); //-5.40274*pow(distance_cm,3)+28.4823*pow(distance_cm,2)-49.7115*distance_cm+31.3444; //From datasheet
//   distance_cm = distance_to_center - distance_cm;  //the distance d = distance from sensor to center - measured distance
  distance_cm = mySensor.distance();
  delay(20);
  x =  (cos(angle) * distance_cm);  //when ir transmit or recieve the layer it obtain some angle  
  y =  (sin(angle) * distance_cm);  
  
}




//Function that maps the value in a float format
float mapFloat(float fval, float val_in_min, float val_in_max, float val_out_min, float val_out_max)
{
  return (fval - val_in_min) * (val_out_max - val_out_min) / (val_in_max - val_in_min) + val_out_min;
}


//Function that writes the value to the SD card
void write_to_SD(float SDx, float SDy, float SDz)
{
  file_values = SD.open(file, FILE_WRITE);
  if (file_values) 
  {
    //file_values.print(x); file_values.print(",");
    //file_values.print(y); file_values.print(",");
    file_values.print(distance_cm); file_values.print(",");
    file_values.println(z);
    file_values.close(); 
  }  
}
