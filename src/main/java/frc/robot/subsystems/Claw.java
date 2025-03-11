package frc.robot.subsystems;

import static frc.robot.Constants.ClawVals.*;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    SparkFlex clawMotor;

         
    public Claw() {
        clawMotor = new SparkFlex(canId, MotorType.kBrushless);
        SparkFlexConfig clawConfig = new SparkFlexConfig();
        clawConfig.smartCurrentLimit(2);
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pickAlgae(){
        clawMotor.set(pullInSpeed);
    }

    public void throwAlgae() {
        clawMotor.set(throwSpeed);
    }

    public void stop() {
        clawMotor.stopMotor();
    }
    
    @Override
    public void periodic(){
    }    
    
}
