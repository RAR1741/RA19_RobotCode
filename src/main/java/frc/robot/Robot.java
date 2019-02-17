/*
 * Copyright (c) 2017-2018 FIRST. All Rights Reserved. Open Source Software - may be modified and
 * shared by FRC teams. The code must be accompanied by the FIRST BSD license file in the root
 * directory of the project.
 */

package frc.robot;

import java.io.File;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;
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
import frc.robot.loggable.LoggableDoubleSolenoid;
import frc.robot.loggable.LoggableNavX;
import frc.robot.loggable.LoggableTalonSRX;
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
  private DigitalInput leftLine;
  private DigitalInput midLine;
  private DigitalInput rightLine;
  private PressureSensor pressureSensor;
  private XboxController driver;
  private XboxController operator;
  private Drivetrain drive;
  private Manipulation manipulation;
  private Scoring scoring;
  private DataLogger dataLogger;
  private LoggableNavX navX;
  private UltrasonicSensor ultrasonicSensor;
  private DoubleSolenoid ledLights;
  private Timer timer;

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

  public void startDataLogging(String mode) {
    String dir = Filesystem.localPath("logs");
    new File(dir).mkdirs();
    TimeZone tz = TimeZone.getTimeZone("EST");
    DateFormat df = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
    df.setTimeZone(tz);
    dataLogger.open(dir + "/log-" + df.format(new Date()) + "_" + mode + ".csv");
    setupDataLogging();
  }

  private void log() {
    dataLogger.log("timer", timer.get());
    dataLogger.log("lineLeft", leftLine.get());
    dataLogger.log("lineCenter", midLine.get());
    dataLogger.log("lineRight", rightLine.get());
    dataLogger.logAll();
    dataLogger.writeLine();
  }

  private void setupDataLogging() {
    dataLogger.addAttribute("timer");
    dataLogger.addAttribute("lineLeft");
    dataLogger.addAttribute("lineCenter");
    dataLogger.addAttribute("lineRight");
    drive.setupLogging(dataLogger);
    manipulation.setupLogging(dataLogger);
    scoring.setupLogging(dataLogger);
    navX.setupLogging(dataLogger);

    dataLogger.writeAttributes();
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

    configureLogging();

    logger.info("Starting drivetrain...");
    drive = new Drivetrain(4, 5, 6, 7);
    logger.info("Drivetrain started");

    logger.info("Starting manipulation...");
    manipulation = new Manipulation(new LoggableTalonSRX(12));
    logger.info("Manipulation started");

    logger.info("Starting scoring...");
    scoring = new Scoring(new LoggableTalonSRX(9), new LoggableTalonSRX(10), new LoggableDoubleSolenoid(2, 6, 7),
        new LoggableDoubleSolenoid(2, 4, 5));
    logger.info("Scoring started");

    compressor = new Compressor(2);
    compressor.start();

    pressureSensor = new PressureSensor(new AnalogInput(0));
    ultrasonicSensor = new UltrasonicSensor(new AnalogInput(1));
    navX = new LoggableNavX(Port.kMXP);

    driver = new XboxController(0);
    operator = new XboxController(1);

    leftLine = new DigitalInput(1);
    midLine = new DigitalInput(2);
    rightLine = new DigitalInput(3);

    // If we're not in the matrix...
    // if (!RuntimeDetector.isSimulation()) {
    // ledLights = new DoubleSolenoid(1, 4, 5);
    // ledLights.set(DoubleSolenoid.Value.kForward);

    // camera = CameraServer.getInstance().startAutomaticCapture();
    // camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    // visionThread = new VisionThread(camera, new MyVisionPipeline(), pipeline -> {
    // if (!pipeline.filterContoursOutput().isEmpty()) {
    // Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    // synchronized (imgLock) {
    // centerX = r.x + (r.width / 2);
    // System.out.println("Camera: " + centerX);
    // }
    // }
    // });
    // visionThread.start();
    // }

    dataLogger = new DataLogger();
    String pathToLogFile = Filesystem.localPath("logs", "log.csv");
    dataLogger.open(pathToLogFile);
    setupDataLogging();

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
    startDataLogging("auto");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Run autonomous code (state machines, etc.)
    log();
  }

  @Override
  public void teleopInit() {
    startDataLogging("teleop");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Run teleop code (interpreting input, etc.)
    // drive.arcadeDrive(driver.getX(GenericHID.Hand.kLeft),
    // driver.getY(GenericHID.Hand.kLeft));
    drive.tankDrive(driver.getY(GenericHID.Hand.kLeft), driver.getY(GenericHID.Hand.kRight));
    manipulation.lift(operator.getY(Hand.kLeft));
    scoring.tilt(operator.getY(Hand.kRight));

    switch (operator.getPOV()) {
    case -1: // None
      break;
    case 0: // d-pad up
      scoring.intakeDown();
      break;
    case 180: // d-pad down
      scoring.intakeUp();
      break;
    default:
      break;
    }

    if (operator.getAButton()) {
      scoring.push();
    } else {
      scoring.retract();
    }

    double upSpeed = 0.60;
    double downSpeed = -0.30;

    if (operator.getXButton()) {
      scoring.tilt(downSpeed);
    } else if (operator.getYButton()) {
      scoring.tilt(upSpeed);
    } else {
      scoring.tilt(0);
    }

    double speedLeft = operator.getTriggerAxis(Hand.kLeft);
    double speedRight = operator.getTriggerAxis(Hand.kRight);

    double collectionSpeed = speedRight - speedLeft;
    scoring.roll(collectionSpeed);
    log();
  }

  @Override
  public void testInit() {
    startDataLogging("test");
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    log();
  }

  @Override
  public void disabledInit() {
    dataLogger.close();
  }
}
