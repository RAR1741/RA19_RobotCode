package frc.robot.loggable;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RuntimeDetector;
import frc.robot.logging.DataLogger;
import frc.robot.logging.Loggable;

public class LoggableTalonSRX extends WPI_TalonSRX implements Loggable {
  public LoggableTalonSRX(int deviceNumber) {
    super(deviceNumber);
  }

  public void setupLogging(DataLogger dl) {
    dl.addAttribute(getName() + "Current");
    dl.addAttribute(getName() + "Voltage");
    dl.addAttribute(getName() + "Value");
    dl.addAttribute(getName() + "Position");
    dl.addAttribute(getName() + "Velocity");
    dl.addAttribute(getName() + "Faults");
  }

  public void log(DataLogger dl) {
    // These values are not implemented in the Snobot simulator, ignore them unless
    // we are on the real robot.
    double current = 0.0;
    double voltage = 0.0;
    Faults faults = new Faults();

    if (!RuntimeDetector.isSimulation()) {
      current = getOutputCurrent();
      voltage = getBusVoltage();
      getFaults(faults);
    }
    dl.log(getName() + "Current", current);
    dl.log(getName() + "Voltage", voltage);
    dl.log(getName() + "Value", get());
    dl.log(getName() + "Position", getSelectedSensorPosition());
    dl.log(getName() + "Velocity", getSelectedSensorVelocity());
    dl.log(getName() + "Faults", faults.toBitfield());
  }
}