package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

/**
 * Drivetrain class for the 2019 robot drivetrain.
 *
 * @author iCodeCoolStuff
 */
public class Drivetrain implements Loggable {
  /**
   * {@value #DEADBAND_LIMIT} The limit for when to stop the motor running if the
   * motor speed is too low.
   */
  private static final double DEADBAND_LIMIT = 0.02;

  private WPI_TalonSRX leftTalon;
  private WPI_TalonSRX leftSlave;
  private WPI_TalonSRX rightTalon;
  private WPI_TalonSRX rightSlave;

  private DigitalInput leftLine;
  private DigitalInput midLine;
  private DigitalInput rightLine;

  private double P;

  /**
   * Constructor
   *
   * @param leftTalon1Id  The CAN id of the first left talon.
   * @param leftTalon2Id  The CAN id of the second left talon.
   * @param rightTalon1Id The CAN id of the first right talon.
   * @param rightTalon2Id The CAN id of the second right talon.
   */
  Drivetrain(WPI_TalonSRX _leftTalon1, WPI_TalonSRX _leftTalon2, WPI_TalonSRX _rightTalon1, WPI_TalonSRX _rightTalon2,
      DigitalInput _leftLine, DigitalInput _midLine, DigitalInput _rightLine) {
    leftTalon = _leftTalon1;
    leftSlave = _leftTalon2;
    rightTalon = _rightTalon1;
    rightSlave = _rightTalon2;

    leftTalon.setInverted(true);
    leftSlave.setInverted(true);

    leftSlave.follow(leftTalon);
    rightSlave.follow(rightTalon);

    leftLine = _leftLine;
    midLine = _midLine;
    rightLine = _rightLine;
  }

  /**
   * Drives the left side of the robot either forward or backward.
   *
   * @param speed the speed at which to drive (ranges from -1.0 to +1.0)
   */
  public void driveLeft(double speed) {
    double sp = deadband(speed);
    leftTalon.set(ControlMode.PercentOutput, sp);
  }

  /**
   * Drives the right side of the robot either forward or backward.
   *
   * @param speed the speed at which to drive (ranges from -1.0 to +1.0)
   */
  public void driveRight(double speed) {
    double sp = deadband(speed);
    rightTalon.set(ControlMode.PercentOutput, sp);
  }

  /**
   * Drives the robot with an arcade style drive
   *
   * @param xDrive The speed to drive the drivetrain in the x direction (ranges
   *               from -1.0 to +1.0)
   * @param yDrive The speed to drive the drivetrain in the y direction (ranges
   *               from -1.0 to +1.0)
   */
  public void arcadeDrive(double xDrive, double yDrive) {
    this.driveLeft(yDrive - xDrive);
    this.driveRight(yDrive + xDrive);
  }

  /**
   * Normalizes the input to 0.0 if it is below the value set by
   * {@link #DEADBAND_LIMIT} This is primarily used for reducing the strain on
   * motors.
   *
   * @param in the input to check
   * @return 0.0 if {@code in} is less than abs(DEADBAND_LIMIT) else {@code in}
   */
  public double deadband(double in) {
    return Math.abs(in) > DEADBAND_LIMIT ? in : 0.0;
  }

  public void setupLogging(DataLogger logger) {
    for (var entry : motorsByName()) {
      logger.addAttribute(entry.getKey() + "Current");
      logger.addAttribute(entry.getKey() + "Voltage");
      logger.addAttribute(entry.getKey() + "Value");
      logger.addAttribute(entry.getKey() + "Position");
      logger.addAttribute(entry.getKey() + "Velocity");
    }
    logger.addAttribute("lineLeft");
    logger.addAttribute("lineCenter");
    logger.addAttribute("lineRight");
  }

  public void log(DataLogger logger) {
    for (var entry : motorsByName()) {
      var talon = entry.getValue();
      logger.log(entry.getKey() + "Current", getOutputCurrent(talon));
      logger.log(entry.getKey() + "Voltage", getBusVoltage(talon));
      logger.log(entry.getKey() + "Value", talon.get());
      logger.log(entry.getKey() + "Position", talon.getSelectedSensorPosition());
      logger.log(entry.getKey() + "Velocity", talon.getSelectedSensorVelocity());
    }
    logger.log("lineLeft", leftLine.get());
    logger.log("lineCenter", midLine.get());
    logger.log("lineRight", rightLine.get());
  }

  /**
   * Uses P control to move.
   *
   * @param error             error from the target
   * @param maxMotorPercent   maximum motor percent
   * @param isDrivingStraight decides if it is turning or going straight (1 =
   *                          straight, -1 = turning)
   */
  public void drivePControl(double error, double maxMotorPercent, int isDrivingStraight) {
    double motorPower = maxMotorPercent * error;
    driveLeft(motorPower);
    driveRight(motorPower * isDrivingStraight);
  }

  /**
   * Uses outside line sensors to follow the line.
   *
   * @param maxMotorPercent maximum motor percent
   */
  public void followLine(double maxMotorPercent) {
    double leftMotorPower = 1;
    double rightMotorPower = 1;
    if (leftLine.get() && !rightLine.get()) {
      leftMotorPower = 0.75;
    }
    if (!leftLine.get() && rightLine.get()) {
      rightMotorPower = 0.75;
    }
    if (leftLine.get() && rightLine.get()) {
      leftMotorPower = 0;
      rightMotorPower = 0;
    }
    driveLeft(leftMotorPower * maxMotorPercent);
    driveRight(rightMotorPower * maxMotorPercent);
  }

  /**
   * Convenience method to return all talons by name for logging purposes.
   *
   * @return list of name/motor entries in order.
   */
  private List<Map.Entry<String, WPI_TalonSRX>> motorsByName() {
    var list = new ArrayList<Map.Entry<String, WPI_TalonSRX>>();

    list.add(Map.entry("LeftFront", this.leftTalon));
    list.add(Map.entry("LeftRear", this.leftSlave));
    list.add(Map.entry("RightFront", this.rightTalon));
    list.add(Map.entry("RightRear", this.rightSlave));

    return list;
  }

  public Double getOutputCurrent(WPI_TalonSRX talon) {
    if (RuntimeDetector.isSimulation()) {
      return 0.0;
    } else {
      return talon.getOutputCurrent();
    }
  }

  public Double getBusVoltage(WPI_TalonSRX talon) {
    if (RuntimeDetector.isSimulation()) {
      return 0.0;
    } else {
      return talon.getBusVoltage();
    }
  }
}
