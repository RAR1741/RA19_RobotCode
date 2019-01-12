/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private Compressor compressor;
    private DoubleSolenoid ds1;
    private DoubleSolenoid ds2;
    private DoubleSolenoid ds3;
    private DoubleSolenoid ds4;
    private XboxController xbc;
    private boolean xButtonState = false;
    private boolean aButtonState = false;
    private boolean bButtonState = false;
    private boolean yButtonState = false;
    private boolean bumperButtonState = false;
    private DigitalInput left;
    private DigitalInput middle;
    private DigitalInput right;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        compressor = new Compressor();
        compressor.start();
        ds1 = new DoubleSolenoid(0, 1);
        ds2 = new DoubleSolenoid(2, 3);
        ds3 = new DoubleSolenoid(4, 5);
        ds4 = new DoubleSolenoid(6, 7);
        ds1.set(DoubleSolenoid.Value.kForward);

        xbc = new XboxController(0);

        left = new DigitalInput(1);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
        case kCustomAuto:
            // Put custom auto code here
            break;
        case kDefaultAuto:
        default:
            // Put default auto code here
            break;
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        if (xbc.getXButton()) {
            xButtonState = !xButtonState;
            if (xButtonState) {
                ds1.set(DoubleSolenoid.Value.kForward);
            } else {
                ds1.set(DoubleSolenoid.Value.kReverse);
            }
        } else if (xbc.getAButton()) {
            aButtonState = !aButtonState;
            if (aButtonState) {
                ds2.set(DoubleSolenoid.Value.kForward);
            } else {
                ds2.set(DoubleSolenoid.Value.kReverse);
            }
        } else if (xbc.getBButton()) {
            bButtonState = !bButtonState;
            if (bButtonState) {
                ds3.set(DoubleSolenoid.Value.kForward);
            } else {
                ds3.set(DoubleSolenoid.Value.kReverse);
            }
        } else if (xbc.getYButton()) {
            yButtonState = !yButtonState;
            if (yButtonState) {
                ds4.set(DoubleSolenoid.Value.kForward);
            } else {
                ds4.set(DoubleSolenoid.Value.kReverse);
            }
        } else if (xbc.getBumper(Hand.kRight)) {
            bumperButtonState = !bumperButtonState;
            if (bumperButtonState) {
                ds1.set(DoubleSolenoid.Value.kForward);
                ds2.set(DoubleSolenoid.Value.kForward);
                ds3.set(DoubleSolenoid.Value.kForward);
                ds4.set(DoubleSolenoid.Value.kForward);
            } else {
                ds1.set(DoubleSolenoid.Value.kReverse);
                ds2.set(DoubleSolenoid.Value.kReverse);
                ds3.set(DoubleSolenoid.Value.kReverse);
                ds4.set(DoubleSolenoid.Value.kReverse);
            }
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
