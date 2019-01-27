package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {
  AnalogInput input;
  private final double CM_PER_VOLT = 102.4;

  public UltrasonicSensor(AnalogInput input) {
    this.input = input;
  }

  /**
   * Get current distance in centimeters.
   * @return double distance in centimeters.
   */
  public double getDistance() {
    double voltage = input.getVoltage();

    return voltage * CM_PER_VOLT; // Convert to cm
  }
}