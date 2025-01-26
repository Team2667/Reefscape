package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import static frc.robot.Constants.ElevatorVals.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private SparkMax followerMotor;
    private SparkMax leaderMotor;

    public Elevator() {

        // Initialize motors. See https://github.com/Team2667/Crescendo-Imported/blob/master/src/main/java/frc/robot/subsystems/Intake.java#L22


        // Setup a SparkBaseConfig object for follower
        //  - Follow leader motor
        //  - Inverted
        // configure followerMotor


        // Setup a SparkBaseConfig object for leaderMotor:
        //  - Set closedLoop pid values to those in Constants.
        //  - Once we add limit switches, we'll need to configure them. This will be done later.

    }

    public void moveUp(){
        // call the set api on leaderMotor to move the motor in an upward direction
    }

    public void moveDown(){
        // call the set api on leaderMotor to moce the motor in a downward direction.
    }

    public void stop(){
        
        // call method on the leaderMotor to stop the motor.
    
    }

    @Override
    public void periodic(){
        leaderMotor.getEncoder().getPosition();
    }
}
