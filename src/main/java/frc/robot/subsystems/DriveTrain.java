package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;
import static frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerveSupport.SwerveModule;
import frc.robot.subsystems.swerveSupport.SwerveModuleConfiguration;
import static frc.robot.Constants.DriveTrainVals.*;

public class DriveTrain extends SubsystemBase {
    Pigeon2 pigeon = new Pigeon2(pigeonId); //CHANGE ME ASAP

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0) * 0.10033 * Math.PI;
    private double headingOffset = 0;

    public static final double MAX_VOLTAGE = 12.0;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public double getRawYaw(){
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getYawAdjusted(){
      //  double yaw = pigeon.getYaw().getValueAsDouble() % 360;
        return pigeon.getRotation2d().getRadians();

    }

    public Rotation2d getGyroscopeRotation() {    
       /* *double yaww=pigeon.getYaw().getValueAsDouble();
        if(yaww<0)
            yaww+=360;
            //yaww=360-yaww;*/
        return pigeon.getRotation2d();
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public void setRotationalOffsetToCurrent(){
        headingOffset = getGyroscopeRotation().getDegrees();
    }

    public void resetIMU() {
        pigeon.setYaw(0);
    }
      
    public DriveTrain(){
        m_frontLeftModule = new SwerveModule(SwerveModuleConfiguration.frontLeftConfig());
        m_frontRightModule = new SwerveModule(SwerveModuleConfiguration.frontRightConfig());
        m_backLeftModule  = new SwerveModule(SwerveModuleConfiguration.backLeftConfig());
        m_backRightModule = new SwerveModule(SwerveModuleConfiguration.backRightConfig());
    }

    public void drive (ChassisSpeeds chassisSpeeds){
        m_chassisSpeeds = chassisSpeeds;
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.setDesiredState(states[2]);
        m_frontRightModule.setDesiredState(states[3]);
        m_backLeftModule.setDesiredState(states[0]);
        m_backRightModule.setDesiredState(states[1]);
    }

    public void drive(SwerveModuleState[] states){
        m_frontLeftModule.setDesiredState(states[2]);
        m_frontRightModule.setDesiredState(states[3]);
        m_backLeftModule.setDesiredState(states[0]);
        m_backRightModule.setDesiredState(states[1]);
    }

    public void moveFieldRelative(double speedMetersPerSecond, double angleInRadians) {
      var moduleState = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angleInRadians));
      m_frontLeftModule.setDesiredState(moduleState);
      m_frontRightModule.setDesiredState(moduleState);
      m_backRightModule.setDesiredState(moduleState);
      m_backLeftModule.setDesiredState(moduleState);
    }

    public void stop() {
        m_frontLeftModule.stop();
        m_frontRightModule.stop();
        m_backRightModule.stop();
        m_backLeftModule.stop();
    }

    @Override
    public void periodic() {
            writeWheelPositions();

    }

    public void resetEncoders()
    {
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition modulePositions[] =
        {
            new SwerveModulePosition(m_frontLeftModule.getWheelPosition(), Rotation2d.fromRadians(m_frontLeftModule.getAbsoluteAngle())),
            new SwerveModulePosition(m_frontRightModule.getWheelPosition(), Rotation2d.fromRadians(m_frontRightModule.getAbsoluteAngle())),
            new SwerveModulePosition(m_backLeftModule.getWheelPosition(), Rotation2d.fromRadians(m_backLeftModule.getAbsoluteAngle())),
            new SwerveModulePosition(m_backRightModule.getWheelPosition(), Rotation2d.fromRadians(m_backRightModule.getAbsoluteAngle()))
        };
        return modulePositions;
    }


    public void writeWheelPositions() {
        SmartDashboard.putNumber("FL-angle (radians)",m_frontLeftModule.getAbsoluteAngle());
        SmartDashboard.putNumber("FR-angle (radians)",m_frontRightModule.getAbsoluteAngle());
        SmartDashboard.putNumber("BL-angle (radians)",m_backLeftModule.getAbsoluteAngle());
        SmartDashboard.putNumber("BR-angle (radians)",m_backRightModule.getAbsoluteAngle());

        SmartDashboard.putNumber("FL-absolute-ec (meters)",m_frontLeftModule.getAbsoluteEncoderReading());
        SmartDashboard.putNumber("FR-absolute-ec (meters)",m_frontRightModule.getAbsoluteEncoderReading());
        SmartDashboard.putNumber("BL-absolute-ec (meters)",m_backLeftModule.getAbsoluteEncoderReading());
        SmartDashboard.putNumber("BR-absolute-ec (meters)",m_backRightModule.getAbsoluteEncoderReading());

        SmartDashboard.putNumber("FL-distance (meters)",m_frontLeftModule.getWheelPosition());
        SmartDashboard.putNumber("FR-distance (meters)",m_frontRightModule.getWheelPosition());
        SmartDashboard.putNumber("BL-distance (meters)",m_backLeftModule.getWheelPosition());
        SmartDashboard.putNumber("BR-distance (meters)",m_backRightModule.getWheelPosition());
    }

    public Trajectory generateTrajectory(Pose2d startPt, List<Translation2d> wayPoints, Pose2d endPt) {
        return TrajectoryGenerator.generateTrajectory(startPt, wayPoints, endPt, createConfig(m_kinematics));
    }

    private TrajectoryConfig createConfig(SwerveDriveKinematics kinematics) {
        var config = new TrajectoryConfig(0.5, 2.0);
        config.setKinematics(kinematics);
        return config;
    }
}