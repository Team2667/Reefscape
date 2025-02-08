package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmVals.*;

public class Arm extends SubsystemBase {
    private SparkFlex armMotor;

    public Arm() {
        // create a new SparkFlex object and assign it to armMotor

        // create a SparkFlexConfig object.
        // In the closedLoop a feedbackSendor property. This needs to be set to a FeedbackSensor.kAlternatOrExternalEncoder enum value. 
        // 
    }

    public void rotateClockwise(){

    }

    public void rotateCounterClockwise(){
        
    }

    public void toProcessorPosition(){
        // On the armMotor, there is something called a closed loop controller. Get that and then 
        // call set reference.
    }

    @Override
    public void periodic(){
        // use smart dashboard to write the position of the arm 
    }
}
