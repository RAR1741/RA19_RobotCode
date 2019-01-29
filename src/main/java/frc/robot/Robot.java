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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
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
  private DigitalInput left;
  private DigitalInput middle;
  private DigitalInput right;
  private PressureSensor pressureSensor;
  private XboxController xbc;
  private Drivetrain drive;

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
    drive.arcadeDrive(xbc.getX(GenericHID.Hand.kLeft), 
                      xbc.getY(GenericHID.Hand.kLeft));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // This isn't typically used in our programs.
  }
}
