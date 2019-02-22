package frc.robot;

import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;

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
   * Uses P control to move the lift.
   *
   * @param error error from the target
   * @param maxMotorPercent maximum motor percent
   */
  public void liftPControl(double error, double maxMotorPercent) {
    double motorPower = maxMotorPercent * error;
    liftTalon.set(motorPower);
  }

  /**
   * Lifts lift to a level.
   *
   * @param level what level it lifts to (1, 2, or 3)
   */
  public void liftLevel(int level) {
    if (level == 1){
      error = (LIFTLEVEL1 - liftTalon.getSelectedSensorPosition()) * P;
    } else if (level == 2) {
      error = (LIFTLEVEL2 - liftTalon.getSelectedSensorPosition()) * P;
    } else if (level == 3) {
      error = (LIFTLEVEL3 - liftTalon.getSelectedSensorPosition()) * P;
    } else {
      error = 0;
    }
    liftPControl(error, 0.8);
  }

  public void setupLogging(DataLogger dl) {
    liftTalon.setupLogging(dl);
  }

  public void log(DataLogger dl) {
    liftTalon.log(dl);
  }
}
