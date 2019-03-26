package frc.robot;

import frc.robot.loggable.LoggableTalonSRX;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.moandjiezana.toml.Toml;

public class Manipulation implements Loggable, Configurable {
  public enum State {
    kStart, kHome, kHold, kMoving, kManual
  }

  private State state;

  private LoggableTalonSRX liftTalon;

  private int tolerance;

  /**
   * Constructor
   *
   * @param liftCanId The CAN id of the talon for the lift motor
   */
  Manipulation(LoggableTalonSRX liftCan) {
    liftTalon = liftCan;
    liftTalon.setName("Lift");
    tolerance = 10;
    state = State.kStart;
  }

  /**
   * auto - autonomous control of the lift.
   */
  public void auto() {
    final double downSpeed = -0.1;
    switch (state) {
    case kStart:
      lift(downSpeed);
      state = State.kHome;
      break;
    case kHome:
      lift(downSpeed);
      if (liftTalon.getSensorCollection().isRevLimitSwitchClosed()) {
        lift(0);
        liftTalon.setSelectedSensorPosition(0);
        liftTalon.set(ControlMode.Position, 0);
        state = State.kHold;
      }
    case kHold:
      // nothing
      break;
    case kMoving:
      if (onTarget()) {
        state = State.kHold;
      }
      break;
    case kManual:
      // nothing
      break;
    }
  }

  /**
   * getState
   *
   * @return state
   */
  public State getState() {
    return state;
  }

  /**
   * Lifts the lift either up or down.
   *
   * @param speed the speed to rotate the motor (ranges from -1.0 to 1.0)
   */
  public void lift(double speed) {
    liftTalon.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Re-configure the manipulation system based on TOML config.
   *
   * @param config the parsed TOML configuration file
   */
  public void configure(Toml config) {
    tolerance = config.getLong("manipulation.tolerance", 10l).intValue();
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.slot0.kP = config.getDouble("manipulation.P", 1.0); // TODO: Come back after tuning values to change "1"
    talonConfig.slot0.kI = config.getDouble("manipulation.I", 0.0);
    talonConfig.slot0.kD = config.getDouble("manipulation.D", 0.0);
    liftTalon.configAllSettings(talonConfig);
    liftTalon.configAllowableClosedloopError(0, tolerance, 0);
    liftTalon.setSensorPhase(true);
  }

  /**
   * Set target position.
   *
   * @param targetPosition wanted position
   */
  public void setSetpoint(int targetPosition) {
    liftTalon.set(ControlMode.Position, targetPosition);
  }

  /**
   * Whether or not we've reached the target position.
   */
  public boolean onTarget() {
    return Math.abs(liftTalon.getClosedLoopError()) <= this.tolerance;
  }

  public void setupLogging(DataLogger dl) {
    liftTalon.setupLogging(dl);
  }

  public void log(DataLogger dl) {
    liftTalon.log(dl);
  }
}
