package frc.robot.subsystems;

import static frc.robot.Constants.ClawVals.*;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    SparkFlex clawMotor;
    SparkFlexConfig throwConfig;
    SparkFlexConfig pickConfig;

         
    public Claw() {
        clawMotor = new SparkFlex(canId, MotorType.kBrushless);
        throwConfig = new SparkFlexConfig();
        throwConfig.smartCurrentLimit(80);
        pickConfig = new SparkFlexConfig();
        pickConfig.smartCurrentLimit(50);
        clawMotor.configure(pickConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pickAlgae(){
        clawMotor.configure(pickConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clawMotor.set(pullInSpeed);
    }

    public void throwAlgae() {
        clawMotor.configure(throwConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clawMotor.set(throwSpeed);
    }

    // TODO: implement this method. It should run the motor in the same direction needed to pick up an Algae.
    // The speed may be different.
    public void dropCoral() {

    }

    public boolean isAlgaePicked() {
        return clawMotor.getOutputCurrent() > maxCurrent;
    }

    public void stop() {
        clawMotor.stopMotor();
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Output Current", clawMotor.getOutputCurrent());
    }    
}
