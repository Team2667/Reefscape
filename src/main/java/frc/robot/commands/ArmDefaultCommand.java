package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDefaultCommand extends Command{
    private XboxController controller;
    private Arm arm;

    public ArmDefaultCommand(Arm arm, XboxController controller) {
        this.controller = controller;
        this.arm = arm;
        addRequirements(arm);
        
    }

    @Override
    public void execute() {
        if (controller.getPOV() == 90) {
            arm.rotateClockwise();
        }
        else if (controller.getPOV() == 270) {
            arm.rotateCounterClockwise();
        }
        else {
            arm.stopMotor();
        }
    }
public void end(boolean isInturupted) {
    arm.stopMotor();
}

}
