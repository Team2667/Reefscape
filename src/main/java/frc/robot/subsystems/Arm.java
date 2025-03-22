package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmVals.*;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;



public class Arm extends SubsystemBase {
    private SparkFlex armMotor;
    private ArmFeedforward ff;
    private static double fullCircleRadians = Math.PI * 2;

    private SparkFlexConfig armConfig;


    
    public Arm() {
        // create a new SparkFlex object and assign it to armMotor
        armMotor = new SparkFlex(canId, MotorType.kBrushless);
    
        armConfig = new SparkFlexConfig();
        armConfig.closedLoop.pid(pV, iV, dV);
        ff = new ArmFeedforward(0, kG, kV, kA);
        armConfig.inverted(true);
        armConfig.closedLoop.outputRange(-0.7, .7);
        armConfig.absoluteEncoder.inverted(true);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armConfig.softLimit.reverseSoftLimit(reverseLimit);
        armConfig.softLimit.forwardSoftLimit(forwardLimit);
        
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum ArmPosition {
        LowReef(0.5),
        HighReef(0.408), //formerly 0.47
        Home(.25),
        Score(.2),
        offGround(0.53),
        OffCoral(0.43);

        ArmPosition(double position){
            this.position = position;
        }

        public double position;
    }

    public void rotateClockwise(){
        armMotor.set(0.25);
    }

    public void rotateCounterClockwise(){
        armMotor.set(-0.25);
    }

    public void runToPosition(TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        armMotor.getClosedLoopController().setReference(setpoint.position, ControlType.kPosition, 
        ClosedLoopSlot.kSlot0, feedforward);
    }
    
    public void stopMotor() {
        armMotor.stopMotor();
    }

    public boolean isAtSetPoint(ArmPosition setPoint){
        return Math.abs(setPoint.position - getArmPosition()) < armMarginOfError;
        
    }

    public double getArmPosition(){
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    public boolean isAtForwardLimit() {
        return false;
    }

    public boolean isAtReverseLimimt() {
        return false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", armMotor.getAbsoluteEncoder().getPosition());
    }
}
