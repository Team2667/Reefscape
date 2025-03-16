package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainAddThrottle extends Command{
    private DriveTrain driveTrain;
    
    public DriveTrainAddThrottle(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.throttleDriveSpeed();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
