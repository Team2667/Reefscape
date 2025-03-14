package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmResetConfig extends Command{

    private Arm arm;
    public ArmResetConfig(Arm arm) {
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.resetConfig();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
