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
  private LoggableDoubleSolenoid intakePivots;

  /**
   * Constructor
   *
   * @param roller       The talon for the roller
   * @param wrist        The talon for the wrist motor
   * @param pushers      The double solenoid for the pushers
   * @param intakePivots The double solenoid for the intake pivot
   */
  Scoring(LoggableTalonSRX roller, LoggableTalonSRX wrist, LoggableDoubleSolenoid pushers,
      LoggableDoubleSolenoid intakePivots) {
    rollerTalon = roller;
    wristTalon = wrist;
    this.pushers = pushers;
    this.intakePivots = intakePivots;

    rollerTalon.setName("Roller");
    wristTalon.setName("Wrist");
    this.pushers.setName("Pushers");
    this.intakePivots.setName("Pivots");
  }

  /**
   * Rotates the roller either forwards or backwards.
   *
   * @param speed the speed to rotate the roller motor (ranges from -1.0 to 1.0)
   */
  public void roll(double speed) {
    // rollerTalon.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Pushes the pneumatic actuators forwards.
   */
  public void push() {
    pushers.set(LoggableDoubleSolenoid.Value.kForward);
  }

  /**
   * Pushes the pneumatic actuators back in.
   */
  public void retract() {
    pushers.set(LoggableDoubleSolenoid.Value.kReverse);
  }

  /**
   * Pushes the intake down.
   */
  public void intakeDown() {
    // intakePivots.set(LoggableDoubleSolenoid.Value.kForward);
  }

  /**
   * Pulls the intake up.
   */
  public void intakeUp() {
    // intakePivots.set(LoggableDoubleSolenoid.Value.kReverse);
  }

  /**
   * Rotates the wrist motor.
   *
   * @param speed the speed to rotate the wrist (ranges from -1.0 to 1.0)
   */
  public void tilt(double speed) {
    wristTalon.set(ControlMode.PercentOutput, speed);
  }

  public void setupLogging(DataLogger dl) {
    wristTalon.setupLogging(dl);
    rollerTalon.setupLogging(dl);
    pushers.setupLogging(dl);
    intakePivots.setupLogging(dl);
  }

  public void log(DataLogger dl) {
    wristTalon.log(dl);
    rollerTalon.log(dl);
    pushers.log(dl);
    intakePivots.log(dl);
  }
}
