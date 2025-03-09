package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorVals.dV;
import static frc.robot.Constants.ElevatorVals.iV;
import static frc.robot.Constants.ElevatorVals.pV;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase{
    SparkFlex clawMotor;
    LaserCan laserCan;
    
    public Claw() {
        // Now: Initialize clawMotor
        // TODO: Define can ID 20, in Constants.
    
        clawMotor = new SparkFlex(20, MotorType.kBrushless);
        SparkFlexConfig clawConfig = new SparkFlexConfig();
        // TODO: Set the closed loop PID values for the Claw motor.
        // See examples in the Arm and/or Elevator subsystems.     
        
        // TODO: Initialize the LaserCan. See https://github.com/GrappleRobotics/LaserCAN/blob/master/docs/example-java.md
    }

    public boolean hasGamePiece() {

        // TODO: return true if the laserCan returns a distance less than some value (3 inches ?)
        // See https://github.com/GrappleRobotics/LaserCAN/blob/master/docs/example-java.md
        return false;
    }

    public void pickAlgae(){
        clawMotor.set(0.25);
        // Set the motor to move in a direction that will bring a alge into the claw
    }

    public void holdAlgae() {
        // TODO: This method should get the current position of the motor and then set reference closed loop to
        // hold the ball.
    }

    public void throwAlgae() {
        // Set the motor to move in direction that will throw the alge
        clawMotor.set(-0.25);
    }

    public void stop() {
        clawMotor.stopMotor();
    }

    
}
