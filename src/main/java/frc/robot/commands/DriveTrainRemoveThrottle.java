package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainRemoveThrottle extends Command{
    private DriveTrain driveTrain;
    
    public DriveTrainRemoveThrottle(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.removeDriveSpeedThrottle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
