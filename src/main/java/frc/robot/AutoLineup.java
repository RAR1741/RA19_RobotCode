package frc.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import frc.vision.MyVisionPipeline;

public class AutoLineup {

  private final int IMG_WIDTH = 640;
  private final int CAM_FOV = 60;
  private final double TARGET_DISTANCE = 40.5;

  private double P; // Needes to be tuned
  private double error;
  private double directDistance;
  private double turnDegree;
  private double pathDistance;

  public enum AutoLineupState {
    LINING_UP,
    TAKE_TARGET_VALUES,
    LINING_UP_2,
    DRIVING,
    TURNING,
    LINE,
    IDLE;
  }

  private AutoLineupState state;
  private Drivetrain drive;
  private UltrasonicSensor sensorL;
  private UltrasonicSensor sensorR;
  private LoggableNavX navx;
  private VisionThread visionThread;
  private double centerX;
  private final Object imgLock = new Object();

  public AutoLineup(Drivetrain drive, UltrasonicSensor sensorL, UltrasonicSensor sensorR, LoggableNavX navx, UsbCamera camera){
    state = AutoLineupState.LINING_UP;
    this.drive = drive;
    this.sensorL = sensorL;
    this.sensorR = sensorR;
    this.navx = navx;

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

  public void run() {
    switch(state) {
      /**
       * Lines the robot up with the target for sensor reading.
       */
      case LINING_UP:
        error = getCameraDegree(centerX) * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -1 || error <= 1){
          state = AutoLineupState.TAKE_TARGET_VALUES;
        }
        break;

      /**
       * Takes sensor readings and calculated target values for later movements.
       */
      case TAKE_TARGET_VALUES:
        directDistance = getDistance();
        turnDegree = getTurnDegree(directDistance);
        pathDistance = getPathDistance(directDistance);
        state = AutoLineupState.LINING_UP_2;
        break;

      /**
       * Lines up robot with new path.
      */
      case LINING_UP_2:
        error = (turnDegree - navx.getYaw()) * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -1 || error  <= 1){
          state = AutoLineupState.DRIVING;
        }
        break;

      /**
       * Drives robot distance of designated path.
       */
      case DRIVING:
        error = (pathDistance - directDistance) * P; // TODO Error won't change while moving
        drive.drivePControl(error, 0.8, 1);
        if (error >= -1 || error <= 1){
          state = AutoLineupState.TURNING;
        }
        break;

      /**
       * Turns robot to face the target.
       */
      case TURNING:
        error = getDistanceDiff() * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -0.5 || error <= 0.5){
          state = AutoLineupState.LINE;
        }
        break;

      /**
       * Follows line.
       */
      case LINE:
        drive.followLine(0.5);
        if (getDistance() <= 31) {
          state = AutoLineupState.IDLE;
        }
        break;

      /**
       * Ends state machine.
       */
      case IDLE:
        break;
    }
  }

  /**
   * Uses two ultrasonic sensors to determine average distance.
   */
  private double getDistance() {
    return (sensorL.getDistance() + sensorR.getDistance()) / 2;
  }

  /**
   * Uses two ultrasonic sensors to determine the difference between the two.
   */
  private double getDistanceDiff() {
    return (sensorL.getDistance() - sensorR.getDistance());
  }
  /*
    double turn = centerX - (IMG_WIDTH/2);
    double cameraDegree = turn/(IMG_WIDTH/CAM_FOV);
    double globalDegree = gyroDegree + cameraDegree;
    double a = Math.cos(globalDegree)*directDistance;
    double b = Math.sqrt(Math.pow(directDistance,2) - Math.pow(a,2));
    double pathDistance = Math.sqrt(Math.pow(a,2) + Math.pow((b - TARGET_DISTANCE),2));
    double turnDegree = Math.acos(a/pathDistance);
  */

  /**
   * Gets the degrees from the center of the camera's vision.
   *
   * @param centerX the center of the vision target
   * @return the number of degrees from the center of the camera's vision
   */
  public double getCameraDegree(double centerX) {
    double turn = centerX - (IMG_WIDTH/2);
    return turn/(IMG_WIDTH/CAM_FOV);
  }

  /**
   * Gets the length in centimeters of the path to a point TARGET_DISTANCE away from the target.\
   *
   * @param directDistance the number of centimeters from the robot to the target
   */
  public double getPathDistance(double directDistance) {
    double gyro = navx.getYaw();
    double a = Math.cos(gyro)*directDistance;
    double b = Math.sqrt(Math.pow(directDistance,2) - Math.pow(a,2));
    return Math.sqrt(Math.pow(a,2) + Math.pow((b - TARGET_DISTANCE),2));
  }

  /**
   * Gets the global degrees the robot must turn to in order to line up with the path.
   *
   * @param centerX the center of the vision target
   * @param directDistance the number of centimeters from the robot to the target
   * @return the global degrees the robot must turn to in order to line up with the path
   */
  public double getTurnDegree(double directDistance){
    double gyro = navx.getYaw();
    double a = Math.cos(gyro)*directDistance;
    double b = Math.sqrt(Math.pow(directDistance,2) - Math.pow(a,2));
    double pathDistance = Math.sqrt(Math.pow(a,2) + Math.pow((b - TARGET_DISTANCE),2));
    return Math.acos(a/pathDistance);
  }
}