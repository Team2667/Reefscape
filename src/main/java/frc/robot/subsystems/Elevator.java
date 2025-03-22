package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import static frc.robot.Constants.ElevatorVals.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private SparkMax followerMotor;
    private SparkMax leaderMotor;
    private double marginOfError = 1.0;

    public enum ElevatorPosition {
        // TODO: We will need to determine which elevator positions we need
        // and there exact values.
        LowerRefPosition(-66),
        MiddleRefPosition(-55),
        TopRefPosition(-69), //formerly 96
        HomePosition(-5),
        offGroundPos(-16.5),
        offCoralPos(-5);
        

        ElevatorPosition(double position){
            this.position = position;
        }

        public double position;
    }

    public Elevator() {
        leaderMotor = new SparkMax(leaderCANId, MotorType.kBrushless);
        followerMotor = new SparkMax(followerCANId, MotorType.kBrushless);

        SparkBaseConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.smartCurrentLimit(40);
        leaderConfig.closedLoop.pid(pV, iV, dV);
        leaderConfig.closedLoop.iMaxAccum(.1);
        leaderConfig.softLimit.reverseSoftLimit(upperLimit);
        leaderConfig.softLimit.reverseSoftLimitEnabled(true);
        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        SparkBaseConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(leaderCANId, true);
        followerConfig.smartCurrentLimit(40);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveToPosition(ElevatorPosition targetPosition) {
        moveToPosition(targetPosition.position);
    }

    public void moveToPosition(double position) {
        leaderMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    public void moveToPosition(double position, double ffValue) {
        leaderMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffValue);
    }

    public double getPosition() {
        return leaderMotor.getEncoder().getPosition();
    }

    public double getVelocity() {
        return leaderMotor.getEncoder().getVelocity();
    }

    public boolean isAtLowerLimit() {
        return leaderMotor.getForwardLimitSwitch().isPressed();
    }

    
    public boolean isAtUpperLimit() {
        return getPosition() >= upperLimit;
    }

    public void ZeroElevator() {
        leaderMotor.getEncoder().setPosition(0.0);
    }

    public boolean isAtPosition(ElevatorPosition targetPosition) {      
        return marginOfError > Math.abs(leaderMotor.getEncoder().getPosition() - targetPosition.position);
    }
   
    public void moveUp(){
        leaderMotor.set(-0.25);
    }

    public void moveDown(){
        leaderMotor.set(0.25);
    }

    public void stop(){
        leaderMotor.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position",  leaderMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Is At Lower Limit", leaderMotor.getForwardLimitSwitch().isPressed());
        
    }
}
