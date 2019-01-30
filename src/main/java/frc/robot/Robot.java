/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.moandjiezana.toml.Toml;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.vision.MyVisionPipeline;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final Logger logger = Logger.getLogger(Robot.class.getName());
  private Toml config;

  private final int IMG_WIDTH = 640;
  private final int IMG_HEIGHT = 480;
  private UsbCamera camera;
  private Compressor compressor;
  private DoubleSolenoid ds1;
  private DoubleSolenoid ds2;
  private DoubleSolenoid ds3;
  private DoubleSolenoid ds4;
  private DoubleSolenoid ds5;
  private DoubleSolenoid ds6;
  private DoubleSolenoid ledLights;
  private XboxController xbc;
  private boolean xButtonState = false;
  private boolean aButtonState = false;
  private boolean bButtonState = false;
  private boolean yButtonState = false;
  private boolean bumperButtonState = false;
  private boolean startButtonState = false;
  private boolean backButtonState = false;
  private DigitalInput left;
  private DigitalInput middle;
  private DigitalInput right;
  private PressureSensor pressureSensor;
  private UltrasonicSensor ultrasonicSensor;

  private VisionThread visionThread;
  private double centerX = 0.0;

  private final Object imgLock = new Object();

  private void configureLogging() {
    try {
      Level logLevel = Level.parse(config.getString("log.level", "INFO"));
      logger.setLevel(logLevel);
    } catch (Exception ex) {
      logger.severe(String.format("Couldn't set log level: %s", ex.getMessage()));
    }
  }

  boolean isSimulation = Objects.equals(System.getProperty("sun.java.command"), "com.snobot.simulator.Main");

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    logger.info("Initializing robot...");

    try {
      String pathToLogFile = "/home/lvuser/deploy/robot.toml";
      logger.info(String.format("Loading log file from \"%s\"", pathToLogFile));
      config = new Toml().read(new File(pathToLogFile));
    } catch (Exception ex) {
      logger.severe(String.format("Couldn't load from file (falling back to empty): %s", ex.getMessage()));
      config = new Toml();
    }

    configureLogging();

    compressor = new Compressor();
    compressor.start();
    ds1 = new DoubleSolenoid(0, 0, 1);
    ds2 = new DoubleSolenoid(0, 2, 3);
    ds3 = new DoubleSolenoid(0, 4, 5);
    ds4 = new DoubleSolenoid(0, 6, 7);

    ds5 = new DoubleSolenoid(1, 0, 1);
    ds6 = new DoubleSolenoid(1, 2, 3);

    pressureSensor = new PressureSensor(new AnalogInput(0));
    ultrasonicSensor = new UltrasonicSensor(new AnalogInput(1));

    ledLights = new DoubleSolenoid(1, 4, 5);
    ledLights.set(DoubleSolenoid.Value.kForward);

    xbc = new XboxController(0);

    left = new DigitalInput(1);

    // If we're not in the matrix...
    if (!isSimulation) {
      camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

      visionThread = new VisionThread(camera, new MyVisionPipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
            System.out.println("Camera: " + centerX);
          }
        }
      });
      visionThread.start();
    }

    pressureSensor = new PressureSensor(new AnalogInput(0));

    logger.info("Robot initialized.");
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
    // This shouldn't be used yet.
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
    logger.info("Entering autonomous mode.");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Run autonomous code (state machines, etc.)
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Run teleop code (interpreting input, etc.)
    System.out.println(String.format("Pressure: %2.2f", pressureSensor.getPressure()));
    if (xbc.getXButtonPressed()) {
      xButtonState = !xButtonState;
      if (xButtonState) {
        ds1.set(DoubleSolenoid.Value.kForward);
      } else {
        ds1.set(DoubleSolenoid.Value.kReverse);
      }
    } else if (xbc.getAButtonPressed()) {
      aButtonState = !aButtonState;
      if (aButtonState) {
        ds2.set(DoubleSolenoid.Value.kForward);
      } else {
        ds2.set(DoubleSolenoid.Value.kReverse);
      }
    } else if (xbc.getBButtonPressed()) {
      bButtonState = !bButtonState;
      if (bButtonState) {
        ds3.set(DoubleSolenoid.Value.kForward);
      } else {
        ds3.set(DoubleSolenoid.Value.kReverse);
      }
    } else if (xbc.getYButtonPressed()) {
      yButtonState = !yButtonState;
      if (yButtonState) {
        ds4.set(DoubleSolenoid.Value.kForward);
      } else {
        ds4.set(DoubleSolenoid.Value.kReverse);
      }
    } else if (xbc.getStartButtonPressed()) {
      startButtonState = !startButtonState;
      if (startButtonState) {
        ds5.set(DoubleSolenoid.Value.kForward);
      } else {
        ds5.set(DoubleSolenoid.Value.kReverse);
      }
    } else if (xbc.getBackButtonPressed()) {
      backButtonState = !backButtonState;
      if (backButtonState) {
        ds6.set(DoubleSolenoid.Value.kForward);
      } else {
        ds6.set(DoubleSolenoid.Value.kReverse);
      }
    } else if (xbc.getBumperPressed(Hand.kRight)) {
      bumperButtonState = !bumperButtonState;
      if (bumperButtonState) {
        ds1.set(DoubleSolenoid.Value.kForward);
        ds2.set(DoubleSolenoid.Value.kForward);
        ds3.set(DoubleSolenoid.Value.kForward);
        ds4.set(DoubleSolenoid.Value.kForward);
        ds5.set(DoubleSolenoid.Value.kForward);
        ds6.set(DoubleSolenoid.Value.kForward);
      } else {
        ds1.set(DoubleSolenoid.Value.kReverse);
        ds2.set(DoubleSolenoid.Value.kReverse);
        ds3.set(DoubleSolenoid.Value.kReverse);
        ds4.set(DoubleSolenoid.Value.kReverse);
        ds5.set(DoubleSolenoid.Value.kReverse);
        ds6.set(DoubleSolenoid.Value.kReverse);
      }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // This isn't typically used in our programs.
  }
}
