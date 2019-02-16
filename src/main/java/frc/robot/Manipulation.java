package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;

public class Manipulation implements Loggable {
  private WPI_TalonSRX liftTalon;

  /**
   * Constructor
   *
   * @param liftCanId The CAN id of the talon for the lift motor
   */
  Manipulation(int liftCanId) {
    liftTalon = new WPI_TalonSRX(liftCanId);
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
    // TODO: Log all talon attributes
    dl.addAttribute("liftInput");
    dl.addAttribute("liftCurrent");
    dl.addAttribute("liftPosition");
    dl.addAttribute("liftFwdLimit");
    dl.addAttribute("liftRevLimit");
  }

  public void log(DataLogger dl) {
    Faults faults = new Faults();
    liftTalon.getFaults(faults);
    dl.log("liftInput", liftTalon.get());
    dl.log("liftCurrent", liftTalon.getOutputCurrent());
    dl.log("liftPosition", liftTalon.getSelectedSensorPosition());
    dl.log("liftFwdLimit", faults.ForwardLimitSwitch);
    dl.log("liftRevLimit", faults.ReverseLimitSwitch);
  }
}
