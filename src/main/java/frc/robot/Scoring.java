package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class for the scoring subsystem used by the 2019 robot
 */
public class Scoring {
    private WPI_TalonSRX rollerTalon;
    private WPI_TalonSRX rotaterTalon;
    private DoubleSolenoid pushers;

    /**
     * Constructor
     * 
     * @param rollerCanId     The CAN id of the talon for the roller motor
     * @param rotateCanId     The CAN id of the talon for the rotater motor
     * @param pushersCanId    The CAN id of the pcm for the pushers
     * @param pushersChannel1 The channel number of the forward channel for the pushers
     * @param pushersChannel2 The channel number of the reverse channel for the pushers
     */
    Scoring(int rollerCanId, int rotaterCanId, int pushersCanId, int pushersChannel1, int pushersChannel2) {
        rollerTalon  = new WPI_TalonSRX(rollerCanId);
        rotaterTalon = new WPI_TalonSRX(rotaterCanId);
        pushers      = new DoubleSolenoid(pushersCanId, pushersChannel1, pushersChannel2);
    }

    /**
     * Rotates the roller either forwards or backwards.
     * @param speed the speed to rotate the motor (ranges from -1.0 to 1.0)
     */
    public void roll(double speed) {
        rollerTalon.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Pushes the pushers forwards.
     */
    public void push() {
        pushers.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Retracts the pushers.
     */
    public void retract() {
        pushers.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Rotates the rotater
     * @param speed the speed to rotate the rotater (ranges from -1.0 to 1.0)
     */
    public void rotate(double speed) {
        rotaterTalon.set(ControlMode.PercentOutput, speed);
    }
}
