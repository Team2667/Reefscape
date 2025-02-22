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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private SparkMax followerMotor;
    private SparkMax leaderMotor;

    public Elevator() {
        leaderMotor = new SparkMax(leaderCANId, MotorType.kBrushless);
        followerMotor = new SparkMax(followerCANId, MotorType.kBrushless);

        SparkBaseConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.closedLoop.pid(pV, iV, dV);
        

        SparkBaseConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(leaderCANId, true);
      //  followerConfig.inverted(true);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveUp(){
        leaderMotor.set(-0.15);
    }

    public void moveDown(){
        leaderMotor.set(0.15);
    }

    public void stop(){
        leaderMotor.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position",  leaderMotor.getEncoder().getPosition());
    }
}
