package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class DrivetrainLift {

  private DoubleSolenoid frontLiftL;
  private DoubleSolenoid frontLiftR;
  private DoubleSolenoid backLift;

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
   * @param backLiftCanId The CAN id of the pcm for the back lift
   * @param backLiftChannel1 The channel number of the forward channel for the back lift
   * @param backLiftChannel2 The channel number of the reverse channel for the back lift
   */
  DrivetrainLift(int rollerCanId, int frontLiftLCanId, int frontLiftLChannel1, int frontLiftLChannel2, int frontLiftRCanId, int frontLiftRChannel1, int frontLiftRChannel2, int backLiftCanId, int backLiftChannel1, int backLiftChannel2) {
    rollerTalon = new WPI_TalonSRX(rollerCanId);
    frontLiftL = new DoubleSolenoid(frontLiftLCanId, frontLiftLChannel1, frontLiftLChannel2);
    frontLiftR = new DoubleSolenoid(frontLiftRCanId, frontLiftRChannel1, frontLiftRChannel2);
    backLift = new DoubleSolenoid(backLiftCanId, backLiftChannel1, backLiftChannel2);
  }

  /**
   * Pushes the front lift out
   */
  public void frontLiftOut() {
    frontLiftL.set(DoubleSolenoid.Value.kForward);
    frontLiftR.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Pulles front lift in
   */
  public void frontLiftIn() {
    frontLiftL.set(DoubleSolenoid.Value.kReverse);
    frontLiftR.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Pushes back lift out
   */
  public void backLiftOut() {
    backLift.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Pulles back lift in
   */
  public void backLiftIn() {
    backLift.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Rotates the roller either forwards or backwards.
   * @param speed the speed to rotate the roller motor (ranges from -1.0 to 1.0)
   */
  public void roll(double speed) {
    rollerTalon.set(ControlMode.PercentOutput, speed);
  }
}