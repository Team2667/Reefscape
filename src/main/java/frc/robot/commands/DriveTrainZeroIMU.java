package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainZeroIMU extends Command{
    
    private DriveTrain driveTrain;
    public DriveTrainZeroIMU(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetIMU();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
