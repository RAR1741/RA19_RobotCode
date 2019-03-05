package frc.robot;

import com.moandjiezana.toml.Toml;

public class InputTransformer implements Configurable {
  private double maxTurn;

  public InputTransformer() {
    maxTurn = 1.0;
  }

  public void configure(Toml config) {
    maxTurn = config.getDouble("input.maxTurn", 1.0);
  }

  double transformDrive(double input) {
    return input * maxTurn;
  }
}