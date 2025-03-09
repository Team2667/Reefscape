package frc.robot.subsystems;

import static frc.robot.Constants.ClawVals.*;
import static frc.robot.Constants.ElevatorVals.dV;
import static frc.robot.Constants.ElevatorVals.iV;
import static frc.robot.Constants.ElevatorVals.pV;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*@Override
    public void robotPeriodic() {
      LaserCan.Measurement measurement = lc.getMeasurement();
      if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        System.out.println("The target is " + measurement.distance_mm + "mm away!");
      } else {
        System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
*/
public class Claw extends SubsystemBase{
    SparkFlex clawMotor;
    LaserCan laserCan;
    
    //define LaserCAN Id = 21
         
    public Claw() {
        // ✓ Now: Initialize clawMotor
        // ✓ TODO: Define can ID 20, in Constants.
        
        clawMotor = new SparkFlex(canId, MotorType.kBrushless);
        SparkFlexConfig clawConfig = new SparkFlexConfig();
        clawConfig.closedLoop.pid(pV, iV, dV);
        clawConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // ✓ TODO: Set the closed loop PID values for the Claw motor.
        // See examples in the Arm and/or Elevator subsystems.     
        
        // TODO: Initialize the LaserCan. See https://github.com/GrappleRobotics/LaserCAN/blob/master/docs/example-java.md
        laserCan = new LaserCan(laserCanId);
    }

    public boolean hasGamePiece() {

        // TODO: return true if the laserCan returns a distance less than some value (3 inches ?)
        // See https://github.com/GrappleRobotics/LaserCAN/blob/master/docs/example-java.md
        return false;
    }

    public void pickAlgae(){
        clawMotor.set(pullInSpeed);
        // ✓ Set the motor to move in a direction that will bring a alge into the claw
    }

    public void holdAlgae() {
        // ✓ TODO: This method should get the current position of the motor and then set reference closed loop to
        // hold the ball.
        double currentPosition = clawMotor.getEncoder().getPosition();
        clawMotor.getClosedLoopController().setReference(currentPosition, ControlType.kPosition);
    }

    public void throwAlgae() {
        // Set the motor to move in direction that will throw the alge
        clawMotor.set(throwSpeed);
    }

    public void stop() {
        clawMotor.stopMotor();
    }
    
    @Override
    public void periodic(){
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        
        SmartDashboard.putNumber("LaserCAN measurment", measurement != null ? measurement.distance_mm : -99);
    }    
    
}
