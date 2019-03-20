package frc.robot;

import com.moandjiezana.toml.Toml;

public class InputTransformer implements Configurable {
  private double maxTurn;
  private double maxClimbSpeed;

  public InputTransformer() {
    maxTurn = 0.6;
    maxClimbSpeed = 0.2;
  }

  public void configure(Toml config) {
    maxTurn = config.getDouble("input.maxTurn", 0.6);
    maxClimbSpeed = config.getDouble("input.maxClimbSpeed", 0.2);
  }

  double transformDrive(double input) {
    return input * maxTurn;
  }

  double transformClimb(double input) {
    return input * maxClimbSpeed;
  }
}