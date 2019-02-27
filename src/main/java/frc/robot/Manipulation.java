package frc.robot;

import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.moandjiezana.toml.Toml;

public class Manipulation implements Loggable {
  private LoggableTalonSRX liftTalon;

  //Lift positions need to be determined
  private final float LIFTLEVEL1 = 0;
  private final float LIFTLEVEL2 = 0;
  private final float LIFTLEVEL3 = 0;

  private double P;
  private double error;

  /**
   * Constructor
   *
   * @param liftCanId The CAN id of the talon for the lift motor
   */
  Manipulation(LoggableTalonSRX liftCan) {
    liftTalon = liftCan;
    liftTalon.setName("Lift");
  }

  /**
   * Lifts the lift either up or down.
   *
   * @param speed the speed to rotate the motor (ranges from -1.0 to 1.0)
   */
  public void lift(double speed) {
    liftTalon.set(ControlMode.PercentOutput, -speed);
  }

  /**
   * Re-configure the manipulation system based on TOML config.
   *
   * @param config the parsed TOML configuration file
   */
  void configure(Toml config) {
  }

  /**
   * Set target position.
   */
  public void setSetpoint(int targetPosition) {
  }

  /**
   * Whether or not we've reached the target position.
   */
  public boolean onTarget() {
  }

  public void setupLogging(DataLogger dl) {
    liftTalon.setupLogging(dl);
  }

  public void log(DataLogger dl) {
    liftTalon.log(dl);
  }
}
