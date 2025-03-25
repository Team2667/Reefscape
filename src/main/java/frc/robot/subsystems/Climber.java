package frc.robot.subsystems;

import static frc.robot.Constants.ClimberVals.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SparkFlex climberMotor;

    public Climber() {
        climberMotor = new SparkFlex(canId, MotorType.kBrushless);
    }

    public void forward() {
        climberMotor.set(forwardSpeed);
    }

    public void reverse() {
        climberMotor.set(reverseSpeed);
    }

    public void stop() {
        climberMotor.stopMotor();
    }

}
