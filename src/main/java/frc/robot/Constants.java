package frc.robot;

public class Constants {

    public static class AvailableSubsystems {
        public static boolean elevatorAvailable = true;
        public static boolean armAvailavble = true;
        public static boolean clawAvailable = true;
        public static boolean driveTrainAvailable = false;
    }

    public static class GameControllerConstants {
        public static int manipulatorGamepadPort = 0;
        public static int driveTrainGamepadPort = 1;
    }

    public static class ElevatorVals {
        public static final int leaderCANId = 11;
        public static final int followerCANId = 10;
        public static final double pV = 0.0008;
        public static final double iV = 0.0;
        public static final double dV = 0;
    }

    public static class ArmVals {
        public static int canId = 30;

        public static final double pV = 0.0008;
        public static final double iV = 0.0;
        public static final double dV = 0;
    }

    public static class ClawVals {
        public static int canId = 20;
        public static double pullInSpeed = .25;
        public static double throwSpeed = .25;
    }
    
    public static class DriveTrainVals {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  private static final double pi=3.141592;
   public static final int pigeonId=31;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0;
  public static final double MAX_INPUT_SPEED = 1; //4.14528;

  // CANIDS
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;
    //public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.9521484375;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.45166015625;
  
  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; 
   //public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.798583984375;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0.35400390625;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
    //public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.6484375;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0.443603515625;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14;
    //public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.444580078125;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.203125;

  public static final double WHEEL_REVOLUTIONS_PER_METER = 3.0;
  public static final double PERCENTAGE_MAX_SPEED = 100.0;

        
    }
    
}
