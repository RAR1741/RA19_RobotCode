package frc.robot;

import frc.robot.Drivetrain;

public class AutoLineup {

  private final int IMG_WIDTH = 640;
  private final int CAM_FOV = 60;
  private final double TARGET_DISTANCE = 200;
  private final double MAX_MOTOR_PERCENT = 0.8;

  private double P;
  private double motorPower;
  private double error;

  public enum AutoLineupState {
    LINING_UP,
    LINING_UP_2,
    DRIVING,
    TURNING,
    IDLE; 
  }
  private AutoLineupState state;
  private Drivetrain drive;

  public AutoLineup(Drivetrain drive){
    state = AutoLineupState.LINING_UP;
    this.drive = drive;
  }

  public void run() {
    switch(state) {
      case LINING_UP:
        error = getCameraDegree(centerX)/2*P;
        motorPower = MAX_MOTOR_PERCENT * P;
        drive.driveLeft(motorPower);
        drive.driveRight(-motorPower);
        if (getCameraDegree(centerX) >= -1 || getCameraDegree(centerX) <= 1){
          double directDistance = getDistance();
          state = AutoLineupState.LINING_UP_2;
        }
        break;


      case LINING_UP_2:
        error = (getTurnDegree(centerX, getDistance()) - getRoll()) * P;
        motorPower = MAX_MOTOR_PERCENT * P;
        drive.driveLeft(motorPower);
        drive.driveRight(-motorPower);
        if (error >= -1 || error  <= 1){
          state = AutoLineupState.DRIVING;
        }
        break;

      case DRIVING:
        error = (getPathDistance(centerX, getDistance()) - getDistance()) * P;
        motorPower = MAX_MOTOR_PERCENT * P;
        drive.driveLeft(motorPower);
        drive.driveRight(motorPower);
        if (error >= -1 || error <= 1){
          state = AutoLineupState.TURNING;
        }
        break;

      case TURNING:
        error = (90 - getRoll()) * P;
        motorPower = MAX_MOTOR_PERCENT * P;
        drive.driveLeft(motorPower);
        drive.driveRight(-motorPower);
        if (error >= -1 || error <= 1){
          state = AutoLineupState.IDLE;
        }
        break;

      case IDLE:
        break;
    }
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
   * @param centerX The center of the vision target
   * @return the number of degrees from the center of the camera's vision
   */
  public double getCameraDegree(double centerX) {
    double turn = centerX - (IMG_WIDTH/2);
    return turn/(IMG_WIDTH/CAM_FOV);
  }

  /**
   * Gets the length in centimeters of the path to a point TARGET_DISTANCE away from the target.\
   * 
   * @param centerX The center of the vision target
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
   * @param centerX The center of the vision target
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