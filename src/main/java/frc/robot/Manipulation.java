package frc.robot;

import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Manipulation implements Loggable {
  private LoggableTalonSRX liftTalon;

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
    liftTalon.set(ControlMode.PercentOutput, speed);
  }

  public void setupLogging(DataLogger dl) {
    dl.addLoggable(liftTalon);
  }

  public void log(DataLogger dl) {
    // logAll will handle it
  }
}
