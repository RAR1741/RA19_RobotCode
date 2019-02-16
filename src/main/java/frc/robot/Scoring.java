package frc.robot;

import frc.robot.loggable.LoggableDoubleSolenoid;
import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class for the scoring subsystem used by the 2019 robot
 */
public class Scoring implements Loggable {
  private LoggableTalonSRX rollerTalon;
  private LoggableTalonSRX wristTalon;
  private LoggableDoubleSolenoid pushers;

  /**
   * Constructor
   *
   * @param roller  The talon for the roller
   * @param wrist   The talon for the wrist motor
   * @param pushers The double solenoid for the pushers
   */
  Scoring(LoggableTalonSRX roller, LoggableTalonSRX wrist, LoggableDoubleSolenoid pushers) {
    rollerTalon = roller;
    wristTalon = wrist;
    this.pushers = pushers;

    rollerTalon.setName("Roller");
    wristTalon.setName("Wrist");
    this.pushers.setName("Pushers");
  }

  /**
   * Rotates the roller either forwards or backwards.
   *
   * @param speed the speed to rotate the roller motor (ranges from -1.0 to 1.0)
   */
  public void roll(double speed) {
    rollerTalon.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Pushes the pneumatic actuators forwards.
   */
  public void intakeDown() {
    pushers.set(LoggableDoubleSolenoid.Value.kForward);
  }

  /**
   * Pushes the pneumatic actuators back in
   */
  public void intakeUp() {
    pushers.set(LoggableDoubleSolenoid.Value.kReverse);
  }

  /**
   * Rotates the wrist motor
   *
   * @param speed the speed to rotate the wrist (ranges from -1.0 to 1.0)
   */
  public void tilt(double speed) {
    wristTalon.set(ControlMode.PercentOutput, speed);
  }

  public void setupLogging(DataLogger dl) {
    dl.addLoggable(wristTalon);
    dl.addLoggable(rollerTalon);
    dl.addLoggable(pushers);
  }

  public void log(DataLogger dl) {
    // No-op, logAll should handle loggables
  }
}
