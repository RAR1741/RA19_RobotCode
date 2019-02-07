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
     * @param tiltCanId       The CAN id of the talon for the tilt motor
     *
     * #TODO find some better name for "tiltMotor" cuz that's not what it does.
     *
     * @param pushersCanId    The CAN id of the pcm for the pushers
     * @param pushersChannel1 The channel number of the forward channel for the pushers
     * @param pushersChannel2 The channel number of the reverse channel for the pushers
     */
    Scoring(int rollerCanId, int rotaterCanId, int tiltCanId, int pushersCanId, int pushersChannel1, int pushersChannel2) {
        rollerTalon    = new WPI_TalonSRX(rollerCanId);
        rotaterTalon   = new WPI_TalonSRX(rotaterCanId);
        tiltTalon      = new WPI_TalonSRX(tiltCanId);
        pushers        = new DoubleSolenoid(pushersCanId, pushersChannel1, pushersChannel2);
    }

    /**
     * Rotates the roller either forwards or backwards.
     * @param speed the speed to rotate the roller motor (ranges from -1.0 to 1.0)
     */
    public void roll(double speed) {
        rollerTalon.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Pushes the pneumatic actuators forwards.
     */
    public void push() {
        pushers.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Pushes the pneumatic actuators back in
     */
    public void insertNameHerePls() {
        pushers.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Extends the scoring subsystem
     */
    public void extend() {}

    /**
     * Retracts the scoring subsystem
     */
    public void retract() {}

    /**
     * Rotates the intake motor
     * @param speed the speed to rotate the rotater (ranges from -1.0 to 1.0)
     */
    public void rotate(double speed) {
        rotaterTalon.set(ControlMode.PercentOutput, speed);
    }
}
