package frc.robot;

public class Constants {

  public static class AvailableSubsystems {
    public static boolean elevatorAvailable = true;
    public static boolean armAvailavble = true;
    public static boolean clawAvailable = true;
    public static boolean driveTrainAvailable = true;
    public static boolean poseEstimatorAvailable = false;
  }

  public static class GameControllerConstants {
    public static int manipulatorGamepadPort = 1;
    public static int driveTrainGamepadPort = 0;
  }

  public static class ElevatorVals {
    public static final int leaderCANId = 15;  // Change this CAN ID
    public static final int followerCANId = 10; //
    public static final double pV = 0.01;
    public static final double iV = 0.00005;
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
    private static final double pi = 3.141592;
    public static final int pigeonId = 31;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0;
    public static final double MAX_INPUT_SPEED = 1; // 4.14528;

    // CANIDS
    public static final int flDriveMotorCANId = 7;
    public static final int flSteerMotorCANId = 8;
    public static final int flSteerEncoderCANId = 13;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.9521484375;
    public static final double flSteerEncoderOffset = -0.45166015625 + 0.5;

    public static final int frDriveMotorCANId = 3;
    public static final int frSteerMotorCANId = 4;
    public static final int frSteerEncoderCANId = 11;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.798583984375;
    public static final double frSteerEncoderOffset = 0.35400390625 + 0.5;

    public static final int blDriveMotorCANId = 1;//5;
    public static final int blSteerMotorCANId = 2;//6;
    public static final int blSteerEncoderCANId = 14;//12;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.6484375;
    public static final double blSteerEncoderOffset = -0.203125 + 0.5;
    

    public static final int brDriveMotorCANId = 5;//1;
    public static final int brSteerMotorCANId = 6;//2;
    public static final int brSteerEncoderCANId = 12;//14;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.444580078125;
    public static final double brSteerEncoderOffset = 0.443603515625 + 0.5;
    

    public static final double WHEEL_REVOLUTIONS_PER_METER = 3.0;
    public static final double PERCENTAGE_MAX_SPEED = 100.0;
  }

  public static class PoseEstimatorVals {
    public static boolean writeDebugVals = false;
  }

}
