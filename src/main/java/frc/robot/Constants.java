package frc.robot;

public class Constants {

    public static class AvailableSubsystems {
        public static boolean elevatorAvailable = true;
        public static boolean armAvailavble = false;
        public static boolean clawAvailable = false;
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

    public static class ClimberVals {


    }

    public static class ClawVals {
        public static int canId = 20;
        public static double pullInSpeed = .25;
        public static double throwSpeed = .25;
    }    
}
