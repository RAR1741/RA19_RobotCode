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

  public enum AutoLineupState {
    LINING_UP,
    LINING_UP_2,
    DRIVING,
    TURNING,
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
      case LINING_UP:
        error = getCameraDegree(centerX) * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -1 || error <= 1){
          directDistance = getDistance();
          state = AutoLineupState.LINING_UP_2;
        }
        break;

      case LINING_UP_2:
        error = (getTurnDegree(centerX, directDistance) - navx.getYaw()) * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -1 || error  <= 1){
          state = AutoLineupState.DRIVING;
        }
        break;

      case DRIVING:
        error = (getPathDistance(centerX, directDistance) - directDistance) * P;
        drive.drivePControl(error, 0.8, 1);
        if (error >= -1 || error <= 1){
          state = AutoLineupState.TURNING;
        }
        break;

      case TURNING:
        error = (90 - navx.getYaw()) * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -1 || error <= 1){
          state = AutoLineupState.IDLE;
        }
        break;

      case IDLE:
        break;
    }
  }

  private double getDistance() {
    return (sensorL.getDistance() + sensorR.getDistance()) / 2;
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
   * @param turn the number of pixels from the center of the camera
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
   * @param centerX the center of the vision target
   * @param directDistance the number of centimeters from the robot to the target
   */
  public double getPathDistance(double centerX, double directDistance) {
    double turn = centerX - (IMG_WIDTH/2);
    double cameraDegree = turn/(IMG_WIDTH/CAM_FOV);
    double a = Math.cos(cameraDegree)*directDistance;
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
  public double getTurnDegree(double centerX, double directDistance){
    double turn = centerX - (IMG_WIDTH/2);
    double cameraDegree = turn/(IMG_WIDTH/CAM_FOV);
    double a = Math.cos(cameraDegree)*directDistance;
    double b = Math.sqrt(Math.pow(directDistance,2) - Math.pow(a,2));
    double pathDistance = Math.sqrt(Math.pow(a,2) + Math.pow((b - TARGET_DISTANCE),2));
    return Math.acos(a/pathDistance);
  }
}