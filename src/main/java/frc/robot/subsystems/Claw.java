package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase{
    SparkMax clawMotor;
    
    public Claw() {
        // Now: Initialize clawMotor

        
        // After we know what kind of limit switches we will be us
        // Create a local SparkMaxConfig object
        // In the limitSwitch properties of this objec:
        // * Enable the reverse limit switch
        // * Set the type of limit switch to either Type.kNormallyClosed or Type.kNormallyOpen
        //   depending on the type of limit switch we end up using.        
    }

    public void pickAlgae(){
        // Set the motor to move in a direction that will bring a alge into the claw
    }

    public void throwAlgae() {
        // Set the motor to move in direction that will throw the alge
    }

    public void stop() {
        // Stop the motor
    }
}
