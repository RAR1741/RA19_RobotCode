package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Manipulation {
  private WPI_TalonSRX liftTalon;

  /**
   * Constructor
   *
   * @param liftCanId The CAN id of the talon for the lift motor
   */
  Manipulation (int liftCanId) {
    liftTalon = new WPI_TalonSRX(liftCanId);
  }

  /**
   * Lifts the lift either up or down.
   * @param speed the speed to rotate the motor (ranges from -1.0 to 1.0)
   */
  public void lift(double speed) {
    liftTalon.set(ControlMode.PercentOutput, speed);
  }
}
