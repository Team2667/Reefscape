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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmVals.*;



public class Arm extends SubsystemBase {
    private SparkFlex armMotor;
    private ArmFeedforward ff;
    private static double fullCircleRadians = Math.PI * 2;
    
    public Arm() {

        // create a new SparkFlex object and assign it to armMotor
        armMotor = new SparkFlex(canId, MotorType.kBrushless);
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.closedLoop.pid(pV, iV, dV);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ff = new ArmFeedforward(ffs, ffg, ffv);
    }

    public enum ArmPosition {

        LowReef(.2);

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

    public void rotateToPosition(ArmPosition armPos) {
        var posInRadians = toArmPositionInRadians(armMotor.getAbsoluteEncoder().getPosition());
        armMotor.getEncoder().getVelocity();
        var currentVelocityInRadians = armMotor.getEncoder().getVelocity() * fullCircleRadians;
        var ffInVolts = ff.calculateWithVelocities(posInRadians, currentVelocityInRadians, 0.0);

        armMotor.getClosedLoopController().setReference(armPos.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffInVolts);
    }
    
    public void stopMotor() {
        armMotor.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
    }
    
    private double toArmPositionInRadians(double sensorValue) {
        return Math.abs(sensorValue - armOffset) * fullCircleRadians;
    }
}
