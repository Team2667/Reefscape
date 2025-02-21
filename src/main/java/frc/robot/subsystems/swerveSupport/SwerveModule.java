package frc.robot.subsystems.swerveSupport;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import static frc.robot.Constants.DriveTrainVals.*;

public class SwerveModule {

    private RelativeEncoderResetTracker relativeEncoderTracker = new RelativeEncoderResetTracker();
    private CANcoder absoluteSteerEncoder;
    private SparkFlex steerMotor;
    private SparkFlex driveMotor;
    private RelativeEncoder steerRelativeEncoder;
    private SwerveModuleConfiguration cfg;
    private static final double MAX_VOLTAGE = 12.0;

    public SwerveModule(SwerveModuleConfiguration config) {
        cfg = config;
        absoluteSteerEncoder = createAbsoluteCanEncoder(cfg.steerAbsoluteEncoderCanId, cfg.steeringOffsetInRadians);
        steerMotor = new SparkFlex(cfg.steerMotorCanId, MotorType.kBrushless);
        double temp_PosConvFactor=2.0 * Math.PI * cfg.steerReduction;
        configureSteerMotor(steerMotor, config.steerP, config.steerI, config.steerD, true, temp_PosConvFactor, temp_PosConvFactor / 60.0);
        
        driveMotor = new SparkFlex(cfg.driveMotorCanId, MotorType.kBrushless);
        double positionConversionFactor = Math.PI * cfg.wheelDiameter * cfg.driveReduction;
        configureDriveMotor(driveMotor, cfg.driveP, cfg.driveI, cfg.driveD, config.driveInverted, positionConversionFactor, positionConversionFactor/60);
        steerRelativeEncoder = steerMotor.getEncoder();
        resetSteerRelativeEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        if (relativeEncoderTracker.isTimeToResetRelativeEncoder(steerRelativeEncoder.getVelocity())){
            resetSteerRelativeEncoder();
        }

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(steerMotor.getEncoder().getPosition()));
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        setDriveVelocity(state.speedMetersPerSecond);
        setReferenceAngle(state.angle.getRadians());
    }

    public void stop(){
        driveMotor.set(0);
        steerMotor.set(0);
    }
      
    public double getAbsoluteAngle() {
        double angle = (absoluteSteerEncoder.getAbsolutePosition().getValueAsDouble())*2*Math.PI;
       // angle %= 2.0 * Math.PI;
       // if (angle < 0.0) {
       //     angle += 2.0 * Math.PI;
       // }
        return angle;
    }

    public double getAbsoluteEncoderReading(){
        return absoluteSteerEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void outputSteerAnglesToDashboard(){
        SmartDashboard.putNumber(getSteerLogLabel("Relative Encoder"), steerMotor.getEncoder().getPosition());
        SmartDashboard.putNumber(getSteerLogLabel("Absolute Encoder"), getAbsoluteAngle());
    }

    public double getWheelPosition(){
        return driveMotor.getEncoder().getPosition();
    }

    private void configureSteerMotor(SparkFlex motor, double proportional, double integral, 
            double derivative, boolean inverted, double positionalConversionFactor, double velocityConversionFactor) {
        var SteerConfig = new SparkFlexConfig();
        SteerConfig
            .inverted(inverted)
            .closedLoop.pid(proportional, integral, derivative);
        SteerConfig.encoder
            .positionConversionFactor(positionalConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        motor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureDriveMotor(SparkFlex motor, double proportional, double integral, 
        double derivative, boolean inverted, double posConverFactor, double velocityConvFactor) {
        var sparkMaxConfig = new SparkFlexConfig();
        sparkMaxConfig
            .inverted(inverted)
            .closedLoop.pid(proportional, integral, derivative);
        sparkMaxConfig.encoder
            .positionConversionFactor(posConverFactor)
            .velocityConversionFactor(velocityConvFactor);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    private void setReferenceAngle(double referenceAngleRadians) {
        steerMotor.getClosedLoopController().setReference(referenceAngleRadians, SparkMax.ControlType.kPosition);
    }

    private void setDriveVelocity(double metersPerSecond) {
        var voltage = (metersPerSecond / MAX_INPUT_SPEED * MAX_VOLTAGE) * (PERCENTAGE_MAX_SPEED / 100);
        SmartDashboard.putNumber(getVelocityLabel("Voltage"), voltage);
        driveMotor.setVoltage(voltage);
     }

    private void resetSteerRelativeEncoder() {
        steerRelativeEncoder.setPosition(getAbsoluteAngle());
    }

    private CANcoder createAbsoluteCanEncoder(int canId, double offset) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        CANcoder encoder = new CANcoder(canId);
        
        encoder.getConfigurator().apply(config);
        return encoder;
    }

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    // This method tells us if its time to reset the relative encoder.
    private class RelativeEncoderResetTracker {
        private final int ENCODER_RESET_ITERATIONS = 500;
        private final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
        private int resetIteration = 0;

        public boolean isTimeToResetRelativeEncoder(double turingVelocity) {
            if (turingVelocity < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                resetIteration += 1;
                if (resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    return true;
                }
            } else {
                resetIteration = 0;
            }
            return false;
        }
    }

    private String getSteerLogLabel(String propertyName) {
       return cfg.label + "-Steer-" + propertyName;
    }

    private String getVelocityLabel(String propertyName) {
        return cfg.label + "-Velocity-" + propertyName;
     }
}