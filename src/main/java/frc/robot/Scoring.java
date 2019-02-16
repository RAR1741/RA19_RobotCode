package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class for the scoring subsystem used by the 2019 robot
 */
public class Scoring {
    private WPI_TalonSRX rollerTalon;
    private WPI_TalonSRX tiltTalon;
    private DoubleSolenoid pushers;

    /**
     * Constructor
     *
     * @param rollerCanId     The CAN id of the talon for the roller motor
     * @param tiltCanId     The CAN id of the talon for the tilt motor
     *     *
     * @param pushersCanId    The CAN id of the pcm for the pushers
     * @param pushersChannel1 The channel number of the forward channel for the pushers
     * @param pushersChannel2 The channel number of the reverse channel for the pushers
     */
    Scoring(int rollerCanId, int tiltCanId, int pushersCanId, int pushersChannel1, int pushersChannel2) {
        rollerTalon    = new WPI_TalonSRX(rollerCanId);
        tiltTalon   = new WPI_TalonSRX(tiltCanId);
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
    public void intakeDown() {
        pushers.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Pushes the pneumatic actuators back in
     */
    public void intakeUp() {
        pushers.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Rotates the tilt motor
     * @param speed the speed to rotate the tilter (ranges from -1.0 to 1.0)
     */
    public void tilt(double speed) {
        tiltTalon.set(ControlMode.PercentOutput, speed);
    }
}
