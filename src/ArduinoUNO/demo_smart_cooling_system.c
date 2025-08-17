// Constants for ADC Reference Voltage and Resolution
#define ADC_VREF_mV    5000.0 // ADC reference voltage in millivolt
#define ADC_RESOLUTION 1024.0 // ADC resolution (10-bit ADC)

#define PIN_LM35 A0 // Define the analog input pin connected to the LM35 temperature sensor

// PID control parameters
#define kp 15   // Proportional gain
#define ki  0   // Integral gain
#define kd  0   // Derivative gain

#define SETPOINT 10  // Desired temperature in Celsius (°C)

// Motor control variables
int motor = 0; // Variable to store the motor speed (0-255)

// PID control variables
float prev_error = 0, del_error = 0, int_error = 0; // Variables to store error, previous error, and integral error for PID

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600); 
  
  // Set pin 3 as an output pin to control the motor
  pinMode(3, OUTPUT); 
}

/**
 * PID control function to calculate motor speed based on temperature error.
 * 
 * @param tempC The current temperature in Celsius.
 * @return The motor speed value to adjust based on PID calculations.
 */
float pid(float tempC)
{
   // Calculate the error between the current temperature and the desired setpoint
   float error = tempC - SETPOINT; 

   // Calculate the change in error (derivative term) for PID control
   del_error = error - prev_error; 

   // Accumulate the error over time (integral term) for PID control
   int_error += error;

   // PID control formula: Output = kp * error + ki * integral error + kd * derivative error
   float out = kp * error + kd * del_error + ki * int_error; 

   // Update the previous error for the next iteration
   prev_error = error; 

   // Return the motor speed based on PID calculations
   return out;
}

void loop() {
  // Read the analog value from the LM35 sensor connected to pin A0
  int adcVal = analogRead(PIN_LM35); 

  // Convert the ADC value to a voltage (in millivolts)
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  
  // Convert the voltage to temperature in Celsius (since LM35 outputs 10 mV per degree Celsius)
  float tempC = milliVolt / 10.0;

  // Print the current temperature to the serial monitor for debugging
  Serial.print(tempC);
  Serial.print(",");
  
  // Use the PID function to calculate the appropriate motor speed based on the temperature
  motor = pid(tempC); 

  // Ensure the motor speed is within the acceptable range of 0 to 255 (PWM value range)
  if (motor > 255)
    motor = 255;
  else if (motor < 0)
    motor = 0;

  // Send the calculated motor speed to pin 3 (motor driver) to adjust the motor speed
  analogWrite(3, motor);  
  
  // Print the motor speed to the serial monitor for debugging/plotting
  Serial.print(motor); 

  // Delay for 500 milliseconds before the next loop iteration
  delay(500);
}
