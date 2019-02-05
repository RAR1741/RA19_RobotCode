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
        error = getCameraDegree(getTurn(centerX))/2*P;
        motorPower = MAX_MOTOR_PERCENT * P;
        drive.driveLeft(motorPower);
        drive.driveRight(-motorPower);
        if (getCameraDegree(getTurn(centerX)) >= -1 || getCameraDegree(getTurn(centerX)) <= 1){
          double directDistance = getDistance();
          state = AutoLineupState.LINING_UP_2;
        }
        break;


      case LINING_UP_2:
        error = (getTurnDegree() - getRoll()) * P;
        motorPower = MAX_MOTOR_PERCENT * P;
        drive.driveLeft(motorPower);
        drive.driveRight(-motorPower);
        if (error >= -1 || error  <= 1){
          state = AutoLineupState.DRIVING;
        }
        break;

      case DRIVING:
        error = (getPathDistance() - getDistance()) * P;
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
   * Gets the number of pixels from the center of the camera.
   * 
   * @param centerX The center of the vision target
   * @return the number of pixels from the center of the camera
   */
  public double getTurn(double centerX) {
    return centerX - (IMG_WIDTH/2);
  }

  /**
   * Gets the degrees from the center of the camera's vision.
   * 
   * @param turn the number of pixels from the center of the camera
   * @return the number of degrees from the center of the camera's vision
   */
  public double getCameraDegree(double turn) {
    return turn/(IMG_WIDTH/CAM_FOV);
  }

  /**
   * Gets the distance in centimeters from the robot to a point.
   * 
   * @param globalDegree the number of degrees of the robot to the target compared to the field
   * @param directDistance the number of centimeters from the robot to the target
   * @return the distance in centimeters from the robot to a point
   */
  public double getA(double cameraDegree, double directDistance){
    return Math.cos(cameraDegree)*directDistance;
  }

  /**
   * Gets the distance in centimeters from a point to the target.
   * 
   * @param directDistance the number of centimeters from the robot to the target
   * @param a the distance in centimeters from the robot to a point
   * @return the distance in centimeters from a point to the target
   */
  public double getB(double directDistance, double a){
    return Math.sqrt(Math.pow(directDistance,2) - Math.pow(a,2));
  }

  /**
   * Gets the length in centimeters of the path to a point TARGET_DISTANCE away from the target.
   * 
   * @param a the distance in centimeters from the robot to a point
   * @param b the distance in centimeters from a point to the target
   * @return the length in centimeters of the path to a point TARGET_DISTANCE away from the target
   */
  public double getPathDistance(double a, double b) {
    return Math.sqrt(Math.pow(a,2) + Math.pow((b - TARGET_DISTANCE),2));
  }

  /**
   * Gets the global degrees the robot must turn to in order to line up with the path.
   * 
   * @param a the distance in centimeters from the robot to a point
   * @param pathDistance the length in centimeters of the path to a point TARGET_DISTANCE away from the target
   * @return the global degrees the robot must turn to in order to line up with the path
   */
  public double getTurnDegree(double a, double pathDistance){
    return Math.acos(a/pathDistance);
  }
}