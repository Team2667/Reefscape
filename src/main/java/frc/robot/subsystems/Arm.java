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
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum ArmPosition {

        // TODO: Determine what positions are for getting Algae from the low and high parts of the reef, for scoring,
        // and for picking up from the ground. Add those positions here.
        LowReef(20.0);

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
        // TODO: implement the method for rotating the arm to the specified position.
        // See example in the elevator subsystem.
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
