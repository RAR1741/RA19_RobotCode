package frc.robot;

import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.moandjiezana.toml.Toml;

public class Manipulation implements Loggable, Configurable {
  private LoggableTalonSRX liftTalon;

  private int tolerance;

  /**
   * Constructor
   *
   * @param liftCanId The CAN id of the talon for the lift motor
   */
  Manipulation(LoggableTalonSRX liftCan) {
    liftTalon = liftCan;
    liftTalon.setName("Lift");
    tolerance = 10;
  }

  /**
   * Lifts the lift either up or down.
   *
   * @param speed the speed to rotate the motor (ranges from -1.0 to 1.0)
   */
  public void lift(double speed) {
    liftTalon.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Re-configure the manipulation system based on TOML config.
   *
   * @param config the parsed TOML configuration file
   */
  public void configure(Toml config) {
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.slot0.kP = config.getDouble("manipulation.P", 1.0); // TODO: Come back after tuning values to change "1"
    talonConfig.slot0.kI = config.getDouble("manipulation.I", 0.0);
    talonConfig.slot0.kD = config.getDouble("manipulation.D", 0.0);
    liftTalon.configAllSettings(talonConfig);
    tolerance = config.getLong("manipulation.tolerance", 10l).intValue();
  }

  /**
   * Set target position.
   *
   * @param targetPosition wanted position
   */
  public void setSetpoint(int targetPosition) {
    // liftTalon.set(ControlMode.Position, targetPosition);
  }

  /**
   * Whether or not we've reached the target position.
   */
  public boolean onTarget() {
    return Math.abs(liftTalon.getClosedLoopError()) <= this.tolerance;
  }

  public void setupLogging(DataLogger dl) {
    liftTalon.setupLogging(dl);
  }

  public void log(DataLogger dl) {
    liftTalon.log(dl);
  }
}
