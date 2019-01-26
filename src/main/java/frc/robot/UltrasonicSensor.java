package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {
  AnalogInput input;

  public UltrasonicSensor(AnalogInput input) {
    this.input = input;
  }

  /**
   * Get current pressure in PSI.
   * @return double pressure in PSI.
   */
  public double getDistance() {
    double voltage = input.getVoltage();

    double distance = voltage/2;

    return distance;
  }
}