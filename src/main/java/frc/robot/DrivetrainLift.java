package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.loggable.LoggableDoubleSolenoid;
import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

public class DrivetrainLift implements Loggable {

  private LoggableDoubleSolenoid frontLift;
  private LoggableDoubleSolenoid backLift;

  private LoggableTalonSRX rollerTalon;

  /**
   * Constructor
   *
   * @param rollerTalon The talon used for rolling forward
   *
   * @param frontLift   The front solenoid
   * @param backLift    The rear solenoid
   */
  DrivetrainLift(LoggableTalonSRX rollerTalon, LoggableDoubleSolenoid frontLift, LoggableDoubleSolenoid backLiftL) {
    this.rollerTalon = rollerTalon;
    this.frontLift = frontLift;
    this.backLift = backLiftL;
  }

  /**
   * Pushes the front lift out
   */
  public void frontLiftOut() {
    frontLift.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Pulls front lift in
   */
  public void frontLiftIn() {
    frontLift.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Pushes back lift out
   */
  public void backLiftOut() {
    backLift.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Pulls back lift in
   */
  public void backLiftIn() {
    backLift.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Rotates the roller either forwards or backwards
   *
   * @param speed the speed to rotate the roller motor (ranges from -1.0 to 1.0)
   */
  public void driveRoll(double speed) {
    rollerTalon.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Setup logging with a data logger.
   *
   * @param logger the data logger
   */
  public void setupLogging(DataLogger logger) {
    rollerTalon.setupLogging(logger);
    frontLift.setupLogging(logger);
    backLift.setupLogging(logger);
  }

  /**
   * Log the state of the drivetrain lift to the supplied data logger.
   *
   * @param logger the data logger
   */
  public void log(DataLogger logger) {
    rollerTalon.log(logger);
    frontLift.log(logger);
    backLift.log(logger);
  }
}