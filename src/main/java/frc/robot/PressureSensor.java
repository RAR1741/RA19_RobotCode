package frc.robot;

import edu.wpi.first.wpilibj.*;

public class PressureSensor {
  AnalogInput input;
  public PressureSensor(AnalogInput input) {
    this.input = input;
  }

  private static final double SUPPLY_VOLTAGE = 5.0;

  public double getPressure() {
    return 250.0 * input.getVoltage() / SUPPLY_VOLTAGE + 25.0;
  }
}