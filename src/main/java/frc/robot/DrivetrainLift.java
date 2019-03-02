package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.loggable.LoggableDoubleSolenoid;
import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

public class DrivetrainLift implements Loggable {

  private LoggableDoubleSolenoid frontLiftL;
  private LoggableDoubleSolenoid frontLiftR;
  private LoggableDoubleSolenoid backLiftL;
  private LoggableDoubleSolenoid backLiftR;

  private LoggableTalonSRX rollerTalon;

  /**
   * Constructor
   *
   * @param rollerTalon The talon used for rolling forward
   *
   * @param frontLiftL  The front left solenoid
   * @param frontLiftR  The front right solenoid
   * @param backLiftL   The rear left solenoid
   * @param backLiftR   The rear right solenoid
   */
  DrivetrainLift(LoggableTalonSRX rollerTalon, LoggableDoubleSolenoid frontLiftL, LoggableDoubleSolenoid frontLiftR,
      LoggableDoubleSolenoid backLiftL, LoggableDoubleSolenoid backLiftR) {
    this.rollerTalon = rollerTalon;
    this.frontLiftL = frontLiftL;
    this.frontLiftR = frontLiftR;
    this.backLiftL = backLiftL;
    this.backLiftR = backLiftR;
  }

  /**
   * Pushes the front lift out
   */
  public void frontLiftOut() {
    frontLiftL.set(DoubleSolenoid.Value.kForward);
    frontLiftR.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Pulls front lift in
   */
  public void frontLiftIn() {
    frontLiftL.set(DoubleSolenoid.Value.kReverse);
    frontLiftR.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Pushes back lift out
   */
  public void backLiftOut() {
    backLiftL.set(DoubleSolenoid.Value.kForward);
    backLiftR.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Pulls back lift in
   */
  public void backLiftIn() {
    backLiftL.set(DoubleSolenoid.Value.kReverse);
    backLiftR.set(DoubleSolenoid.Value.kReverse);
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
    frontLiftL.setupLogging(logger);
    frontLiftR.setupLogging(logger);
    backLiftL.setupLogging(logger);
    backLiftR.setupLogging(logger);
  }

  /**
   * Log the state of the drivetrain lift to the supplied data logger.
   *
   * @param logger the data logger
   */
  public void log(DataLogger logger) {
    rollerTalon.log(logger);
    frontLiftL.log(logger);
    frontLiftR.log(logger);
    backLiftL.log(logger);
    backLiftR.log(logger);
  }
}