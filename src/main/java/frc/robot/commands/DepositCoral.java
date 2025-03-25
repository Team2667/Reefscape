package frc.robot.commands;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Claw;


public class DepositCoral extends Command{

    private Claw claw; 
    
    public void initializeClaw(Claw claw) {
        this.claw = claw;
        this.addRequirements(claw);
        
    }
    public void initialize() {
        claw.dropCoral();
    }
    
    public void stop() {
        claw.stop();
    }
}
