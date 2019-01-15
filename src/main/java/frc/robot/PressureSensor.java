package frc.robot;

import edu.wpi.first.wpilibj.*;

public class PressureSensor {
  AnalogInput input;
  public PressureSensor(AnalogInput input) {
    this.input = input;
  }

  public float getPressure() {
    return 0.0f;
  }
}