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
import java.util.LinkedList;
import java.util.List;
import java.util.TimeZone;
import java.util.logging.Level;
import java.util.logging.Logger;
import com.moandjiezana.toml.Toml;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.loggable.LoggableXboxController;
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
  private LoggableXboxController driver;
  private LoggableXboxController operator;
  private Drivetrain drive;
  private Manipulation manipulation;
  private Scoring scoring;
  private DrivetrainLift climber;
  private DataLogger dataLogger;
  private LoggableNavX navX;
  private UltrasonicSensor ultrasonicSensor;
  private DoubleSolenoid ledLights;
  private InputTransformer inputTransformer;
  private Timer timer;
  private List<Configurable> configurables;
  private LoggableTalonSRX manipTalon;

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
    // long startTime = System.nanoTime();
    // dataLogger.log("timer", timer.get());
    // dataLogger.log("lineLeft", leftLine.get());
    // dataLogger.log("lineCenter", midLine.get());
    // dataLogger.log("lineRight", rightLine.get());
    // driver.log(dataLogger);
    // operator.log(dataLogger);
    // drive.log(dataLogger);
    // manipulation.log(dataLogger);
    // scoring.log(dataLogger);
    // climber.log(dataLogger);
    // navX.log(dataLogger);

    // dataLogger.writeLine();
    // long endTime = System.nanoTime();

    // long duration = (endTime - startTime);
    // double durationInMs = (double) duration / 1000000.0;

    // System.out.printf("Log duration: %f ms\n", durationInMs);
  }

  private void setupDataLogging() {
    dataLogger.addAttribute("timer");
    dataLogger.addAttribute("lineLeft");
    dataLogger.addAttribute("lineCenter");
    dataLogger.addAttribute("lineRight");
    driver.setupLogging(dataLogger);
    operator.setupLogging(dataLogger);
    drive.setupLogging(dataLogger);
    manipulation.setupLogging(dataLogger);
    scoring.setupLogging(dataLogger);
    climber.log(dataLogger);
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

    configurables = new LinkedList<Configurable>();
    readConfiguration();

    configureLogging();

    logger.info("Starting drivetrain...");
    drive = new Drivetrain(4, 5, 6, 7);
    logger.info("Drivetrain started");

    manipTalon = new LoggableTalonSRX(12);
    logger.info("Starting manipulation...");
    manipulation = new Manipulation(manipTalon);
    logger.info("Manipulation started");
    configurables.add(manipulation);

    logger.info("Starting scoring...");
    scoring = new Scoring(new LoggableTalonSRX(9), new LoggableTalonSRX(10), new LoggableDoubleSolenoid(2, 6, 7),
        new LoggableDoubleSolenoid(2, 4, 5));
    logger.info("Scoring started");

    logger.info("Starting climber...");
    climber = new DrivetrainLift(new LoggableTalonSRX(8), // roller
        new LoggableDoubleSolenoid(3, 4, 5), new LoggableDoubleSolenoid(3, 6, 7));
    logger.info("Climber started.");

    inputTransformer = new InputTransformer();

    compressor = new Compressor(3);
    compressor.start();

    pressureSensor = new PressureSensor(new AnalogInput(0));
    ultrasonicSensor = new UltrasonicSensor(new AnalogInput(1));
    navX = new LoggableNavX(Port.kMXP);

    driver = new LoggableXboxController(0);
    driver.setName("driver");
    operator = new LoggableXboxController(1);
    operator.setName("operator");

    leftLine = new DigitalInput(1);
    midLine = new DigitalInput(2);
    rightLine = new DigitalInput(3);

    // If we're not in the matrix...
    if (!RuntimeDetector.isSimulation()) {
      // ledLights = new DoubleSolenoid(1, 4, 5);
      // ledLights.set(DoubleSolenoid.Value.kForward);

      camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setFPS(20);
      camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

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
    }

    dataLogger = new DataLogger();
    String pathToLogFile = Filesystem.localPath("logs", "log.csv");
    dataLogger.open(pathToLogFile);
    setupDataLogging();
    reloadConfiguration();

    logger.info("Robot initialized.");
  }

  public void readConfiguration() {
    try {
      String pathToTomlFile = Filesystem.localDeployPath("robot.toml");
      logger.info(String.format("Loading log file from \"%s\"", pathToTomlFile));
      config = new Toml().read(new File(pathToTomlFile));
    } catch (Exception ex) {
      logger.severe(String.format("Couldn't load from file (falling back to empty): %s", ex.getMessage()));
      config = new Toml();
    }
  }

  public void reloadConfiguration() {
    for (Configurable configurable : configurables) {
      configurable.configure(this.config);
    }
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
    reloadConfiguration();
    startDataLogging("auto");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Run autonomous code (state machines, etc.)
    humanControl();
    log();
  }

  @Override
  public void teleopInit() {
    reloadConfiguration();
    startDataLogging("teleop");
  }

  /**
   * Factor out all human controls into a common function so we can drive during
   * the sandstorm period.
   */
  private void humanControl() {
    // Run teleop code (interpreting input, etc.)
    double turnInput = driver.getX(Hand.kRight);
    double speedInput = driver.getY(Hand.kLeft);

    // If we're in climb mode (either button is pressed)
    if (driver.getBButton() || driver.getYButton()) {
      // Instead control secondary drive
      speedInput = inputTransformer.transformClimb(speedInput);
      climber.driveRoll(speedInput);

      if (driver.getBButton()) {
        climber.backLiftOut();
      } else {
        climber.backLiftIn();
      }

      if (driver.getYButton()) {
        climber.frontLiftOut();
      } else {
        climber.frontLiftIn();
      }
    } else {
      // Normal drive mode
      climber.frontLiftIn();
      climber.backLiftIn();
      climber.driveRoll(0.0);

      if (driver.getTriggerAxis(Hand.kRight) < 0.5) {
        turnInput = inputTransformer.transformDrive(turnInput);
        speedInput = inputTransformer.transformDrive(speedInput);
      }
      if (driver.getTriggerAxis(Hand.kLeft) >= 0.5) {
        turnInput = inputTransformer.transformClimb(turnInput);
        turnInput = inputTransformer.transformClimb(speedInput);
      }
      if (driver.getBumper(GenericHID.Hand.kRight)) {
        speedInput = -speedInput;
      }
    }

    drive.arcadeDrive(turnInput, speedInput);

    scoring.tilt(operator.getY(Hand.kRight));

    switch (operator.getPOV()) {
    case -1: // None
      manipulation.lift(operator.getY(Hand.kLeft));
      break;
    case 0: // d-pad up
      // manipulation.setSetpoint(1000);
      // scoring.push();
      break;
    case 180: // d-pad down
      // manipulation.setSetpoint(2000);
      // scoring.retract();
      break;
    default:
      break;
    }

    if (operator.getAButton()) {
      scoring.retract();
    } else {
      scoring.push();
    }

    if (operator.getBButton()) {
      scoring.intakeDown();
    } else {
      scoring.intakeUp();
    }

    double speedLeft = operator.getTriggerAxis(Hand.kLeft);
    double speedRight = operator.getTriggerAxis(Hand.kRight);

    double collectionSpeed = speedRight - speedLeft;
    scoring.roll(collectionSpeed);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    humanControl();
    log();
    // System.out.printf("encoder ticks: %d\n",
    // manipTalon.getSelectedSensorPosition());

    // System.out.printf("Limit Switch fwd: %b\n",
    // manipTalon.getSensorCollection().isFwdLimitSwitchClosed());
    // System.out.printf("Limit Switch rev: %b\n",
    // manipTalon.getSensorCollection().isRevLimitSwitchClosed());
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

  @Override
  public void disabledPeriodic() {
    // System.out.printf("Limit Switch fwd: %b\n",
    // manipTalon.getSensorCollection().isFwdLimitSwitchClosed());
    // System.out.printf("Limit Switch rev: %b\n",
    // manipTalon.getSensorCollection().isRevLimitSwitchClosed());
  }
}
