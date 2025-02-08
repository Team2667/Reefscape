package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmVals.*;

public class Arm extends SubsystemBase {
    private SparkFlex armMotor;

    public Arm() {
        // create a new SparkFlex object and assign it to armMotor
        armMotor = new SparkFlex(30, MotorType.kBrushless);
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.closedLoop.pid(pV, iV, dV);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        

        
        // In the closedLoop there is a feedbackSendor property. 
        // This needs to be set to a FeedbackSensor.kAlternatOrExternalEncoder enum value. 
        // Set the PID values. See elevator for an example-done
        // 
    }

    public void rotateClockwise(){
        armMotor.set(0.25);
    }

    public void rotateCounterClockwise(){
        armMotor.set(-0.25);
    }
    
    public void stopMotor() {
        armMotor.stopMotor();
    }

    public void toProcessorPosition(){

        armMotor.getClosedLoopController();
        // On the armMotor, there is something called a closed loop controller. Get that and then 
        // call set reference.
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
    }    
}
