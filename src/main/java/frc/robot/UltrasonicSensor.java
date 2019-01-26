package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {
  AnalogInput input;

  public UltrasonicSensor(AnalogInput input) {
    this.input = input;
  }

  // private static final double VOLTAGE_AT_ZERO = 0.0;
  // private static final double VOLTAGE_AT_MAX = 5.0;
  // private static final double MAX_DISTANCE = 450.0;

  /**
   * Get current pressure in PSI.
   * @return double pressure in PSI.
   */
  public double getDistance() {
    double voltage = input.getVoltage();

    double distance = voltage/2;
    // double range = VOLTAGE_AT_MAX - VOLTAGE_AT_ZERO;
    // double distance = (voltage - VOLTAGE_AT_ZERO) * MAX_DISTANCE / range;

    return distance;
  }
}