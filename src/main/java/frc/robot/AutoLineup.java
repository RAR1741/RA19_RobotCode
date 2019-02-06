package frc.robot;

public class AutoLineup {

  private final int IMG_WIDTH = 640;
  private final int CAM_FOV = 60;
  private final double TARGET_DISTANCE = 200;

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
  private UltrasonicSensor sensor;
  private LoggableNavX navx;

  public AutoLineup(Drivetrain drive, UltrasonicSensor sensor, LoggableNavX navx){
    state = AutoLineupState.LINING_UP;
    this.drive = drive;
    this.sensor = sensor;
    this.navx = navx;
  }

  public void run(double centerX) {
    switch(state) {
      case LINING_UP:
        error = getCameraDegree(centerX) * P;
        drive.drivePControl(error, 0.8, -1);
        if (error >= -1 || error <= 1){
          directDistance = sensor.getDistance();
          state = AutoLineupState.LINING_UP_2;
        }
        break;

      case LINING_UP_2:
        error = (getTurnDegree(centerX, directDistance) - navx.getRoll()) * P;
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
        error = (90 - navx.getRoll()) * P;
        drive.drivePControl(error, 0.8, -1);
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