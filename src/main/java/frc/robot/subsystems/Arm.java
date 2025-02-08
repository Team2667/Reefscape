package frc.robot.subsystems;

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
        armMotor = new SparkFlex(canId,MotorType.kBrushless);

        // create a SparkFlexConfig object.
        // In the closedLoop a feedbackSendor property. This needs to be set to a FeedbackSensor.kAlternatOrExternalEncoder enum value. 
        // 
        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
    }

}
