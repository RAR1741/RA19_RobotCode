package frc.robot;

import edu.wpi.first.wpilibj.*;

public class PressureSensor {
  AnalogInput input;
  public PressureSensor(AnalogInput input) {
    this.input = input;
  }

  private static final double VOLTAGE_AT_EMPTY = 0.50;
  private static final double VOLTAGE_AT_FULL = 2.70;
  private static final double MAX_PRESSURE = 110.0;

  public double getPressure() {
    double voltage = input.getVoltage();
    double range = VOLTAGE_AT_FULL - VOLTAGE_AT_EMPTY;
    double pressure = (voltage - VOLTAGE_AT_EMPTY) * MAX_PRESSURE / range;

    return pressure;
  }
}