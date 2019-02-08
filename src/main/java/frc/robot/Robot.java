/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
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
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.logging.DataLogger;
import frc.robot.Filesystem;
import frc.robot.LoggableNavX;
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
  private DataLogger dataLogger;
  private LoggableNavX navX;
  private UltrasonicSensor ultrasonicSensor;
  private DoubleSolenoid ledLights;
  private Timer timer;
  private double centerX = 0.0;
  private AutoLineup autoLineup;

  private boolean aButtonState = false;

  private void configureLogging() {
    try {
      Level logLevel = Level.parse(config.getString("log.level", "INFO"));
      logger.setLevel(logLevel);
    } catch (Exception ex) {
      logger.severe(String.format("Couldn't set log level: %s", ex.getMessage()));
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    logger.info("Initializing robot...");

    logger.info("Starting timer...");
    timer = new Timer();
    timer.start();
    logger.info("Timer started");

    try {
      String pathToTomlFile = Filesystem.localDeployPath("robot.toml");
      logger.info(String.format("Loading log file from \"%s\"", pathToTomlFile));
      config = new Toml().read(new File(pathToTomlFile));
    } catch (Exception ex) {
      logger.severe(String.format("Couldn't load from file (falling back to empty): %s", ex.getMessage()));
      config = new Toml();
    }

    logger.info("Starting drivetrain...");
    drive = new Drivetrain(4,5,6,7);
    logger.info("Drivetrain started");

    configureLogging();

    compressor = new Compressor(2);
    compressor.start();

    pressureSensor = new PressureSensor(new AnalogInput(0));
    ultrasonicSensor = new UltrasonicSensor(new AnalogInput(1));
    navX = new LoggableNavX(Port.kMXP);

    xbc = new XboxController(0);

    left = new DigitalInput(1);

    // If we're not in the matrix...
    if (!RuntimeDetector.isSimulation()) {
      ledLights = new DoubleSolenoid(1, 4, 5);
      ledLights.set(DoubleSolenoid.Value.kForward);

      camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(IMG_WIDTH, IMG_HEIGHT);


    }


    dataLogger = new DataLogger();
    String pathToLogFile = Filesystem.localPath("logs", "log.csv");
    dataLogger.open(pathToLogFile);
    dataLogger.addAttribute("timer");
    dataLogger.addLoggable(drive);
    dataLogger.addLoggable(navX);
    dataLogger.setupLoggables();
    dataLogger.writeAttributes();

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
    autoLineup = new AutoLineup(drive, ultrasonicSensor, navX, camera);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Run autonomous code (state machines, etc.)
    autoLineup.run();
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
    if (xbc.getAButtonPressed()) {
      aButtonState = !aButtonState;
      if (aButtonState) {
        autoLineup.run();
      }
    }

    dataLogger.log("timer", timer.get());
    drive.log(dataLogger);
    navX.log(dataLogger);
    dataLogger.writeLine();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // This isn't typically used in our programs.
  }
}
