package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class DrivetrainLift {

  private DoubleSolenoid frontLiftL;
  private DoubleSolenoid frontLiftR;
  private DoubleSolenoid backLiftL;
  private DoubleSolenoid backLiftR;

  private WPI_TalonSRX rollerTalon;

  /**
   * Constructor
   *
   * @param rollerCanId The CAN id of the talon for the roller motor
   *
   * @param frontLiftLCanId The CAN id of the pcm for the front left lift
   * @param frontLiftLChannel1 The channel number of the forward channel for the front left lift
   * @param frontLiftLChannel2 The channel number of the reverse channel for the front left lift
   *
   * @param frontLiftRCanId The CAN id of the pcm for the front right lift
   * @param frontLiftRChannel1 The channel number of the forward channel for the front right lift
   * @param frontLiftRChannel2 The channel number of the reverse channel for the front right lift
   *
   * @param backLiftLCanId The CAN id of the pcm for the back left lift
   * @param backLiftLChannel1 The channel number of the forward channel for the back left lift
   * @param backLiftLChannel2 The channel number of the reverse channel for the back left lift
   *
   * @param backLiftRCanId The CAN id of the pcm for the back right lift
   * @param backLiftRChannel1 The channel number of the forward channel for the back right lift
   * @param backLiftRChannel2 The channel number of the reverse channel for the back right lift
   */
  DrivetrainLift(int rollerCanId, int frontLiftLCanId, int frontLiftLChannel1, int frontLiftLChannel2, int frontLiftRCanId, int frontLiftRChannel1, int frontLiftRChannel2, int backLiftLCanId, int backLiftLChannel1, int backLiftLChannel2, int backLiftRCanId, int backLiftRChannel1, int backLiftRChannel2) {
    rollerTalon = new WPI_TalonSRX(rollerCanId);
    frontLiftL = new DoubleSolenoid(frontLiftLCanId, frontLiftLChannel1, frontLiftLChannel2);
    frontLiftR = new DoubleSolenoid(frontLiftRCanId, frontLiftRChannel1, frontLiftRChannel2);
    backLiftL = new DoubleSolenoid(backLiftLCanId, backLiftLChannel1, backLiftLChannel2);
    backLiftR = new DoubleSolenoid(backLiftRCanId, backLiftRChannel1, backLiftRChannel2);
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
}