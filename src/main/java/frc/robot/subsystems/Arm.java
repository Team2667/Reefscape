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

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;



public class Arm extends SubsystemBase {
    private SparkFlex armMotor;
    private ArmFeedforward ff;
    private static double fullCircleRadians = Math.PI * 2;


    private double arm_pV = 0.0;
    private double arm_iV = 0.0;
    private double arm_dV = 0.0;
    private double arm_kG = 0.0;
    private double arm_kV = 0.0;
    private double arm_kA = 0.0;
    
    private  double arm_kSVolts = kSVolts;
    private  double arm_kGVolts = kGVolts;
    private  double arm_kVVoltSecondPerRad = kVVoltSecondPerRad;
    private  double arm_kAVoltSecondSquaredPerRad = kAVoltSecondSquaredPerRad;
    private SparkFlexConfig armConfig;


    
    public Arm() {
        arm_pV = pV;
        arm_iV = iV;
        arm_dV = dV;
        arm_kG = kG;
        arm_kV = kV;
        arm_kA = kA;

        // create a new SparkFlex object and assign it to armMotor
        armMotor = new SparkFlex(canId, MotorType.kBrushless);
    
        armConfig = new SparkFlexConfig();
        armConfig.closedLoop.pid(arm_pV, arm_iV, arm_dV);
        ff = new ArmFeedforward(0, arm_kG, arm_kV, arm_kA);
        armConfig.inverted(true);
        armConfig.closedLoop.outputRange(-0.7, .7);
        armConfig.absoluteEncoder.inverted(true);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        //TODO: PL02: armConfig has a softLimit property. This property allows you to set the value
        // for the forward and reverse limit switches. It also allows you to enable the limit switches.
        // * Set the forwardSoftLimit to forward limit.
        // * Set the reverseSoftLimit to reverseLimit.
        // * Enable both limit switches.
        // See elevator constructor for an example.
        
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        printOutVals();
    }

    public enum ArmPosition {

        LowReef(0.5),
        HighReef(0.408), //formerly 0.47
        Home(.25),
        Score(.2),
        offGround(0.53),
        OffCoral(0.43);
        

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

    public double getArmPosition(){
        return armMotor.getEncoder().getPosition();
    }

    // TODO PL02: Return true if the arm position is greater than or equal to forwardLimit
    public boolean isAtForwardLimit() {
        return false;
    }

    // TODO PL02: Return true if the arm position is less than or equal to reverseLimit
    public boolean isAtReverseLimimt() {
        return false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", armMotor.getAbsoluteEncoder().getPosition());
        getValuesFromSmartDashBoard();
     // SmartDashboard.getNumber("SVolts-revised", arm_kSVolts);
    }

    public void printOutVals(){
        SmartDashboard.putNumber("pV", arm_pV);
        SmartDashboard.putNumber("iV", arm_iV);
        SmartDashboard.putNumber("dV", arm_dV);
        SmartDashboard.putNumber("kG", arm_kG);
        SmartDashboard.putNumber("kV", arm_kV);
        SmartDashboard.putNumber("kA", arm_kA);
    }

    public void resetConfig(){
        arm_pV = SmartDashboard.getNumber("pV", arm_pV);
        arm_iV = SmartDashboard.getNumber("iV", arm_iV);
        arm_dV = SmartDashboard.getNumber("dV", arm_dV);
        arm_kG = SmartDashboard.getNumber("kG", arm_kG);
        arm_kV = SmartDashboard.getNumber("kV", arm_kV);
        arm_kA = SmartDashboard.getNumber("kA", arm_kA);

        armConfig.closedLoop.pid(arm_pV, arm_iV, arm_dV);
        ff = new ArmFeedforward(0, arm_kG, arm_kV, arm_kA);
        SmartDashboard.putString("Config updated", LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));
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
