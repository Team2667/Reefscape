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

    // This command should be very simmilar to ElevatorDefaultCommand accept that instead of using 
    // the D-Pad to make the motor move, we will be using the triggers on the controller.
    // The constructor for this command will take 2 objects, an Arm and an XboxController.
    // 
    // The command will implement the execute and end method simmilar to ElevatorDefaultCommand.
    // However, instead of checking for depad input, it will check to see if the triggers have been pressed.
    // pressing the right trigger will make it go one way, the left trigger will make it go the oppisite.
    //
}
