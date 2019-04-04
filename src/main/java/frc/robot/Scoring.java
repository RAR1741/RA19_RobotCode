package frc.robot;

import frc.robot.loggable.LoggableDoubleSolenoid;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class for the scoring subsystem used by the 2019 robot
 */
public class Scoring implements Loggable {
  private LoggableDoubleSolenoid pushers;
  private LoggableDoubleSolenoid intakePivots;

  /**
   * Constructor
   *
   * @param pushers      The double solenoid for the pushers
   * @param intakePivots The double solenoid for the intake pivot
   */
  Scoring(LoggableDoubleSolenoid pushers, LoggableDoubleSolenoid intakePivots) {

    this.pushers = pushers;
    this.intakePivots = intakePivots;

    this.pushers.setName("Pushers");
    // this.intakePivots.setName("Pivots");
  }

  /**
   * Extends the forebar (fourbar?) outwards.
   */
  public void extend() {
    pushers.set(LoggableDoubleSolenoid.Value.kReverse);
  }

  /**
   * Retracts the forebar (fourbar?) inwards.
   */
  public void retract() {
    pushers.set(LoggableDoubleSolenoid.Value.kForward);
  }

  /**
   * Extend "finger" on scoring subsystem.
   */
  public void fingerDown() {
    intakePivots.set(LoggableDoubleSolenoid.Value.kReverse);
  }

  /**
   * Retract "finger" on scoring subsystem.
   */
  public void fingerUp() {
    intakePivots.set(LoggableDoubleSolenoid.Value.kForward);
  }

  public void setupLogging(DataLogger dl) {
    pushers.setupLogging(dl);
    // intakePivots.setupLogging(dl);
  }

  public void log(DataLogger dl) {
    pushers.log(dl);
    // intakePivots.log(dl);
  }

  public boolean isExtended() {
    return pushers.get() == LoggableDoubleSolenoid.Value.kForward;
  }
}
