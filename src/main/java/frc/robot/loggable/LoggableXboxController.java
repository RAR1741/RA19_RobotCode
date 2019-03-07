package frc.robot.loggable;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

public class LoggableXboxController extends XboxController implements Loggable {
  private String name;

  public LoggableXboxController(final int port) {
    super(port);
  }

  public void setName(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return this.name;
  }

  public void setupLogging(DataLogger logger) {
    logger.addAttribute(getName() + "LeftX");
    logger.addAttribute(getName() + "LeftY");
    logger.addAttribute(getName() + "RightX");
    logger.addAttribute(getName() + "RightY");

    logger.addAttribute(getName() + "A");
    logger.addAttribute(getName() + "B");
    logger.addAttribute(getName() + "X");
    logger.addAttribute(getName() + "Y");
  }

  public void log(DataLogger logger) {
    logger.log(getName() + "LeftX", this.getX(Hand.kLeft));
    logger.log(getName() + "LeftY", this.getY(Hand.kLeft));
    logger.log(getName() + "RightX", this.getX(Hand.kRight));
    logger.log(getName() + "RightY", this.getY(Hand.kRight));

    logger.log(getName() + "A", this.getAButton());
    logger.log(getName() + "B", this.getBButton());
    logger.log(getName() + "X", this.getXButton());
    logger.log(getName() + "Y", this.getYButton());
  }
}