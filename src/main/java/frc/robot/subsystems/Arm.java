package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmVals.*;



public class Arm extends SubsystemBase {
    private SparkFlex armMotor;
    private ArmFeedforward ff;
    private static double fullCircleRadians = Math.PI * 2;


    public  double arm_pV = pV;
    public  double arm_iV = iV;
    public  double arm_dV = dV;
    
    public  double arm_kSVolts = kSVolts;
    public  double arm_kGVolts = kGVolts;
    public  double arm_kVVoltSecondPerRad = kVVoltSecondPerRad;
    public  double arm_kAVoltSecondSquaredPerRad = kAVoltSecondSquaredPerRad;
    
    public Arm() {

        // create a new SparkFlex object and assign it to armMotor
        armMotor = new SparkFlex(canId, MotorType.kBrushless);
    
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.closedLoop.pid(pV, iV, dV);
        armConfig.inverted(true);
        armConfig.closedLoop.outputRange(-0.7, .7);
       //armConfig.encoder.inverted(true);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ff = new ArmFeedforward(0, kG, kV,kA);
    }

    public enum ArmPosition {

        LowReef(0.5);

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

    public void runToPosition(TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        armMotor.getClosedLoopController().setReference(setpoint.position, ControlType.kPosition, 
        ClosedLoopSlot.kSlot0, feedforward);
    }
    
    public void stopMotor() {
        armMotor.stopMotor();
    }

    public boolean isAtSetPoint(ArmPosition setPoint){
        return Math.abs(setPoint.position - getArmPosition()) < armMarginOfError;
        
    }

    private double getArmPosition(){
        return armMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", armMotor.getAbsoluteEncoder().getPosition());
        getValuesFromSmartDashBoard();
     // SmartDashboard.getNumber("SVolts-revised", arm_kSVolts);
    }

    public void printOutVals(){
      /*   public static final double pV = 0.05;
        public static final double iV = 0.0;
        public static final double dV = 0;
        public static final double ff = 0;
    
        public static final double kSVolts = 0;
        public static final double kGVolts = 0.33;
        public static final double kVVoltSecondPerRad = 6.24;
        public static final double kAVoltSecondSquaredPerRad = 0.04;*/
        SmartDashboard.putNumber("SVolts", kSVolts);
        SmartDashboard.putNumber("GVolts", kGVolts);
        SmartDashboard.putNumber("kVVoltSecondPerRad", kVVoltSecondPerRad);
        SmartDashboard.putNumber("kAVoltSecondSquaredPerRad", kAVoltSecondSquaredPerRad);
        
        SmartDashboard.putNumber("Arm-P", pV);
        SmartDashboard.putNumber("Arm-P", iV);
        SmartDashboard.putNumber("Arm-P", dV);
    }

    void getValuesFromSmartDashBoard(){
        arm_kSVolts = SmartDashboard.getNumber("SVolts", kSVolts);
        arm_kGVolts = SmartDashboard.getNumber("GVolts", kGVolts);
        arm_kVVoltSecondPerRad = SmartDashboard.getNumber("kVVoltSecondPerRad", kVVoltSecondPerRad);
        arm_kAVoltSecondSquaredPerRad = SmartDashboard.getNumber("kAVoltSecondSquaredPerRad", kAVoltSecondSquaredPerRad);
        
        arm_pV = SmartDashboard.getNumber("Arm-P", pV);
        arm_iV = SmartDashboard.getNumber("Arm-P", iV);
        arm_dV = SmartDashboard.getNumber("Arm-P", dV);

        ff = new ArmFeedforward(
            arm_kSVolts, arm_kGVolts,
            arm_kVVoltSecondPerRad, arm_kAVoltSecondSquaredPerRad);
    }
}
