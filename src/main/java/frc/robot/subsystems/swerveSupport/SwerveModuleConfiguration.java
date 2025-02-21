package frc.robot.subsystems.swerveSupport;

import frc.robot.Constants;
import static frc.robot.Constants.DriveTrainVals.*;

public class SwerveModuleConfiguration {
    public int steerMotorCanId;
    public int steerAbsoluteEncoderCanId;
    public double steeringOffsetInRadians;
    public int driveMotorCanId;
    public double revolutionsPerMeter;
    public double steerP = 0.5;
    public double steerI = 0.0;
    public double steerD = 0.0;
    public double driveP = 1;
    public double driveI = 0.0;
    public double driveD = 0.0;
    /*
    public double steerP = 1.0;
    public double steerI = 0.0;
    public double steerD = 0.1;
    public double driveP = 1.0;
    public double driveI = 0.0;
    public double driveD = 0.1;
       */
    public double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);
    public String label;
    public boolean driveInverted = false;
    public double wheelDiameter = 0.1008106757429582;
    public double driveReduction = 1/6.75;

    public static SwerveModuleConfiguration frontRightConfig() {
        var moduleConfig = new SwerveModuleConfiguration();
        moduleConfig.steerMotorCanId = flSteerMotorCANId;
        moduleConfig.steerAbsoluteEncoderCanId = flSteerEncoderCANId;
        moduleConfig.steeringOffsetInRadians = flSteerEncoderOffset;
        moduleConfig.driveMotorCanId = flDriveMotorCANId;
        moduleConfig.revolutionsPerMeter = WHEEL_REVOLUTIONS_PER_METER;
        moduleConfig.label = "Front Left";

        return moduleConfig;
    }

    public static SwerveModuleConfiguration frontLeftConfig() {
        var moduleConfig = new SwerveModuleConfiguration();
        moduleConfig.steerMotorCanId = frSteerMotorCANId;
        moduleConfig.steerAbsoluteEncoderCanId = frSteerEncoderCANId;
        moduleConfig.steeringOffsetInRadians = frSteerEncoderOffset;
        moduleConfig.driveMotorCanId = frDriveMotorCANId;
        moduleConfig.revolutionsPerMeter = WHEEL_REVOLUTIONS_PER_METER;
        moduleConfig.label = "Front Right";

        return moduleConfig;
    }

    public static SwerveModuleConfiguration backRightConfig() {
        var moduleConfig = new SwerveModuleConfiguration();
        moduleConfig.steerMotorCanId = brSteerMotorCANId;
        moduleConfig.steerAbsoluteEncoderCanId = brSteerEncoderCANId;
        moduleConfig.steeringOffsetInRadians = brSteerEncoderOffset;
        moduleConfig.driveMotorCanId = brDriveMotorCANId;
        moduleConfig.revolutionsPerMeter = WHEEL_REVOLUTIONS_PER_METER;
        moduleConfig.label = "Back Right";
        return moduleConfig;
    }

    public static SwerveModuleConfiguration backLeftConfig() {
        var moduleConfig = new SwerveModuleConfiguration();
        moduleConfig.steerMotorCanId = blSteerMotorCANId;
        moduleConfig.steerAbsoluteEncoderCanId = blSteerEncoderCANId;
        moduleConfig.steeringOffsetInRadians = blSteerEncoderOffset;
        moduleConfig.driveMotorCanId = blDriveMotorCANId;
        moduleConfig.revolutionsPerMeter = WHEEL_REVOLUTIONS_PER_METER;
        moduleConfig.label = "Back Left";

        return moduleConfig;
    }
}
