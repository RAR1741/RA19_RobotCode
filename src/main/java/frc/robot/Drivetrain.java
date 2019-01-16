package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Drivetrain {
    
    private final double DEADBAND_LIMIT = 0.02;

    private WPI_TalonSRX leftTalon;
    private WPI_TalonSRX leftSlave;
    private WPI_TalonSRX rightTalon;
    private WPI_TalonSRX rightSlave;

    Drivetrain() {
        leftTalon  = new WPI_TalonSRX(0);
        leftSlave  = new WPI_TalonSRX(1);
        rightTalon = new WPI_TalonSRX(2);
        rightSlave = new WPI_TalonSRX(3);

        leftSlave.follow(leftTalon);
        rightSlave.follow(rightTalon);
    }
    /**
     * Drives the left side of the robot either forward or backward.
     * @param speed the speed at which to drive (ranges from -1.0 to +1.0)
     */
    public void driveLeft(double speed) {
        double sp = deadband(speed);
        leftTalon.set(ControlMode.PercentOutput, sp);
    }

    /**
     * Drives the right side of the robot either forward or backward.
     * @param speed the speed at which to drive (ranges from -1.0 to +1.0)
     */
    public void driveRight(double speed) {
        double sp = deadband(speed);
        rightTalon.set(ControlMode.PercentOutput, sp);
    }

    /**
     * Normalizes the input to 0.0 if it is below the value set by {@link #DEADBAND_LIMIT}
     * This is primarily used for reducing the strain on motors.
     * @param in the input to check
     * @return 0.0 if {@code in} is less than abs(DEADBAND_LIMIT) else {@code in}
     */
    public double deadband(double in) {
        return Math.abs(in) > DEADBAND_LIMIT ? in : 0.0;
    }
}
