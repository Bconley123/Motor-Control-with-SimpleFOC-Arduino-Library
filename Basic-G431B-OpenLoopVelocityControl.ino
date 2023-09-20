#include <SimpleFOC.h>

// Motor instance
// The number of pole pairs is set to 11 for the BLDC motor
BLDCMotor motor = BLDCMotor(11);

// Driver instance for 6-PWM signals
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// Current sense instance
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Target velocity
float target_velocity = 0;

// Instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

void setup() {

  // Driver config
  // Power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();

  // Link the motor and the driver
  motor.linkDriver(&driver);

  // Link current sense and the driver
  currentSense.linkDriver(&driver);

  // Current sensing
  currentSense.init();
  // No need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // Limiting motor movements
  motor.voltage_limit = 3;  // [V]

  // Open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // Initialize motor
  motor.init();

  // Add target command T and voltage limit command L
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  // Open-loop velocity movement
  motor.move(target_velocity);

  // User communication
  command.run();
}
