package frc.robot.loggable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

public class LoggableDoubleSolenoid extends DoubleSolenoid implements Loggable {
  /**
   * Constructor. Uses the default PCM ID (defaults to 0).
   *
   * @param forwardChannel The forward channel number on the PCM (0..7).
   * @param reverseChannel The reverse channel number on the PCM (0..7).
   */
  public LoggableDoubleSolenoid(final int forwardChannel, final int reverseChannel) {
    super(forwardChannel, reverseChannel);
  }

  /**
   * Constructor.
   *
   * @param moduleNumber   The module number of the solenoid module to use.
   * @param forwardChannel The forward channel on the module to control (0..7).
   * @param reverseChannel The reverse channel on the module to control (0..7).
   */
  public LoggableDoubleSolenoid(final int moduleNumber, final int forwardChannel, final int reverseChannel) {
    super(moduleNumber, forwardChannel, reverseChannel);
  }

  @Override
  public void setupLogging(DataLogger dl) {
    dl.addAttribute(getName() + "State");
  }

  @Override
  public void log(DataLogger dl) {
    dl.log(getName() + "State", get().toString());
  }
}