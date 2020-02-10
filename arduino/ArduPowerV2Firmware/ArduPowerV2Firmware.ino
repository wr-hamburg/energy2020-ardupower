/*
 *  ArduPower v2 Firmware
 *  Code by Daniel Guenther Bremer
 *  Part of bachelor's thesis "Optimizing ArduPower"
 */

#define ANALOG_PINS 16

// Declaration of pins
// All possible pins for type-sensing (ordered)
int sensor_type_input_pin[] = {12, 11, 10, 9, 8, 7, 6, 4, 5, 3, 16, 17, 2, 14, 15};
int sensor_type[16];
int sensor_pin[] = {A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
// Probe voltage ADC pin in Automatic Configuration
int voltage_sens_pin = A15;
// Multiplexer (MUX) enable pin
int E_mux_pin = 42;
// MUX channel selection pins
int A_pin[] = {18, 19, 20, 21};
// Array that contains list of connected Probes, their types and voltages
int* active_sensors = (int *) malloc((ANALOG_PINS-1)*sizeof(int));
int* active_sensors_types = (int *) malloc((ANALOG_PINS-1)*sizeof(int));
int* active_sensors_voltages = (int *) malloc((ANALOG_PINS-1)*sizeof(int));
// Pins for the RGB-LED
int r = 22, g = 26, b = 28;
int readData;
int sendcontainer = 0x00;
// #Connected_Probes
int sensor_pin_count = sizeof(sensor_type_input_pin) / sizeof(int);
// Measurement enable/disable var; 0 == false, 1 == true
short measure = 0;
// Receiving buffer
char recv_msg;

/*
 * Function ran once at the program start to set up the device and variables
 * Used to prepare all pins and start the serial connection
 */
void setup() {
  Serial.begin(115200, SERIAL_6N1);
  setPinMode(sensor_pin, ANALOG_PINS, INPUT);
  setPinMode(A_pin, 4, OUTPUT);             // setup channel sel
  pinMode(r, OUTPUT);                          // setup LED
  pinMode(b, OUTPUT);                          // setup LED
  pinMode(g, OUTPUT);                          // setup LED
  pinMode(24, OUTPUT);                         // setup LED
  digitalWrite(24, LOW);                       // setup LED
  pinMode(E_mux_pin, OUTPUT);           // setup MUX enable
  digitalWrite(E_mux_pin, LOW);             // Disable the MUX
  // Read initial configuration
  sensorConfiguration(active_sensors, active_sensors_types, active_sensors_voltages);
  digitalWrite(r, HIGH);
  digitalWrite(g, LOW);
  digitalWrite(b, LOW);
}

/*
 * Main loop
 * Provide data to monitor if measurement is running
 */
void loop() {
  if (measure == 1) {
    collectAndSendData();
  }
}

/*
 * Handling for arriving serial data
 * Triggered automatically by an interrupt
 * Reads data, modifies the control variable measure depending
 * on the input and or handles funciton calls
 */
void serialEvent() {
  recv_msg = Serial.read();
  switch (recv_msg) {
    case '4':                  // Send a set of data once
      measure = 0;
      collectAndSendData();
      break;
    case '2':                  // Send probe configuration
      measure = 0;
      Serial.write(sensor_pin_count);
      for (int i = 0; i < sensor_pin_count; i++) {
        Serial.write(active_sensors_types[i]);
        Serial.write(active_sensors_voltages[i]);
        //min(active_sensors) = A0 = 54 => mapping to 0-14
        Serial.write(active_sensors[i]-54);
      }
      break;
    case '1':                   // start measurement
      Serial.write(1);          // send feedback
      measure = 1;              // enable measurement
      digitalWrite(r, LOW);     // switch LED to green
      digitalWrite(g, HIGH);
      break;
    case '0':                   // abort measurement
      Serial.write(0);          // send feedback
      measure = 0;              // disable measurement
      digitalWrite(g, LOW);     // switch LED to red
      digitalWrite(r, HIGH);
      break;
    case '3':                // rerun Automatic Configuration
      measure = 0;
      sensorConfiguration(active_sensors, active_sensors_types, active_sensors_voltages);
      Serial.write(3);          // send feedback
      break;
    default:                    // do nothing if undef
      break;
  }
}

/*
 * Helper function to set an array of pins to a desired mode
 */
void setPinMode(int pins[], int len, int mode) {
  for (int i = 0; i < len; i++) {
    pinMode(pins[i], mode);
    if (mode == 1) {
      digitalWrite(pins[i], LOW);
    }
  }
}

/*
 * The Automatic Configuration function
 * Read information of the probes
 * Parameters are global accessible memory to save data
 */
void sensorConfiguration(int* sensorptr, int* typeptr, int* voltageptr) {
  int areadVal = 0;
  int active[ANALOG_PINS];
  int types[ANALOG_PINS];
  int voltages[ANALOG_PINS];
  int active_count = 0;

  //Enable the MUX
  digitalWrite(E_mux_pin, HIGH);

  //Set sensor_type_input_pins to INPUT_PULLUP
  setPinMode(sensor_type_input_pin, sensor_pin_count, INPUT_PULLUP);

  for (int i = 0; i < ANALOG_PINS; i++) {
    setMuxChannel(i+1);          // offset of +1 because of shield-design, MUX-Channel 1 is not used
    areadVal = analogRead(voltage_sens_pin);  // read voltage of probed line
    if (areadVal > 716) {           // Voltage > 3,5V => 12V
      voltages[active_count] = 1;
      active[active_count] = sensor_pin[i];
    } else if (areadVal > 306) {    // Voltage > 1,5V => 5V
      voltages[active_count] = 2;
      active[active_count] = sensor_pin[i];
    } else if (areadVal > 184) {    // Voltage > 0,9V => 3,3V
      voltages[active_count] = 3;
      active[active_count] = sensor_pin[i];
    } else if (areadVal > 50) {     // Voltage > 0,24V => 230V
      voltages[active_count] = 4;
      active[active_count] = sensor_pin[i];
    } else {                        // Voltage 0V => NC
      continue;                     // don't mark active
    }
    
    // Read the sensor type
    if (digitalRead(sensor_type_input_pin[i]) == LOW) {  //sensor_type-line connected to ground => 20A sensor
      types[active_count] = 1;
    } else {                  //sensor_type-line connected to 5V => 40A sensor
      types[active_count] = 0;
    }
    active_count++;
  }
  
  // reset to INPUT to prevent leak currents
  setPinMode(sensor_type_input_pin, sensor_pin_count, INPUT);
  
  // disable the MUX
  digitalWrite(E_mux_pin, LOW);
  
  // Save the results
  sensor_pin_count = active_count;
  //truncate the tmp arrays and save data
  for(int i = 0; i < active_count; i++) {
    sensorptr[i] = active[i];
    typeptr[i] = types[i];
    voltageptr[i] = voltages[i];
  }
}

/*
 * Control the multiplexer, write the desired channel to the control pins
 */
void setMuxChannel(int channel) {
  digitalWrite(A_pin[0], (channel & (1 << 0)) == 1 ? HIGH : LOW);
  digitalWrite(A_pin[1], (channel & (1 << 1)) == 2 ? HIGH : LOW);
  digitalWrite(A_pin[2], (channel & (1 << 2)) == 4 ? HIGH : LOW);
  digitalWrite(A_pin[3], (channel & (1 << 3)) == 8 ? HIGH : LOW);
}

/*
 * Read the output of every connected probe and send the result to the monitoring computer
 */
void collectAndSendData() {
  for(int i = 0; i < sensor_pin_count; i++) {
    readData = analogRead(active_sensors[i]);  //Read probe's output
    sendcontainer = (readData>>5) & 0b011111;  //store HIGH bits
    if(i == 0) {                               //set SYNC bit
      sendcontainer |= 0b100000;
    }
    Serial.write(sendcontainer);               //write HIGH bits
    Serial.write(readData & 0b011111);         //write LOW bits
  }
}
