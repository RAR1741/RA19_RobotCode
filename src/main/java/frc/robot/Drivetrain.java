package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Drivetrain class for the 2019 robot drivetrain.
 * @author iCodeCoolStuff
 */
public class Drivetrain {
    
    /**{@value #DEADBAND_LIMIT} The limit for when to stop the motor running if the motor speed is too low.*/
    private final double DEADBAND_LIMIT = 0.02;

    private WPI_TalonSRX leftTalon;
    private WPI_TalonSRX leftSlave;
    private WPI_TalonSRX rightTalon;
    private WPI_TalonSRX rightSlave;

    /**
     * Constructor
     *
     * @param leftTalon1Id  The CAN id of the first left talon.
     * @param leftTalon2Id  The CAN id of the second left talon.
     * @param rightTalon1Id The CAN id of the first right talon.
     * @param rightTalon2Id The CAN id of the second right talon.
     */
    Drivetrain(int leftTalon1Id, int leftTalon2Id, int rightTalon1Id, int rightTalon2Id) {
        leftTalon  = new WPI_TalonSRX(leftTalon1Id);
        leftSlave  = new WPI_TalonSRX(leftTalon2Id);
        rightTalon = new WPI_TalonSRX(rightTalon1Id);
        rightSlave = new WPI_TalonSRX(rightTalon2Id);

        leftTalon.setInverted(true);
        leftSlave.setInverted(true);

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
     * Drives the robot with an arcade style drive
     * 
     * @param xDrive The speed to drive the drivetrain in the x direction (ranges from -1.0 to +1.0)
     * @param yDrive The speed to drive the drivetrain in the y direction (ranges from -1.0 to +1.0)
     */
    public void arcadeDrive(double xDrive, double yDrive) {
        this.driveLeft(yDrive - xDrive);
        this.driveRight(yDrive + xDrive);
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
