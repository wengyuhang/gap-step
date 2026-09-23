# PX4 versioned ActuatorMotors, MESSAGE_VERSION=0.
uint32 MESSAGE_VERSION = 0
uint64 timestamp
uint64 timestamp_sample
uint16 reversible_flags
uint8 ACTUATOR_FUNCTION_MOTOR1 = 101
uint8 NUM_CONTROLS = 12
float32[12] control
