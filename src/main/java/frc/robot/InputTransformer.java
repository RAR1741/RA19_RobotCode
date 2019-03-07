package frc.robot;

import com.moandjiezana.toml.Toml;

public class InputTransformer implements Configurable {
  private double maxTurn;

  public InputTransformer() {
    maxTurn = 0.6;
  }

  public void configure(Toml config) {
    maxTurn = config.getDouble("input.maxTurn", 0.6);
  }

  double transformDrive(double input) {
    return input * maxTurn;
  }
}