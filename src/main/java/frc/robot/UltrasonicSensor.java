package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {
  AnalogInput input;

  public UltrasonicSensor(AnalogInput input) {
    this.input = input;
  }

  /**
   * Get current distance in centimeters.
   * @return double distance in centimeters.
   */
  public double getDistance() {
    double voltage = input.getVoltage();

    double distance = voltage/1024;
    distance /= 10;  // Convert from mm to cm

    return distance;
  }
}