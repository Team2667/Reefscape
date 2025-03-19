package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDefaultCommand extends Command {
    private XboxController controller;
    private Elevator elevator;
    private double elevatorPos;

    public ElevatorDefaultCommand(Elevator elevator, XboxController controller) {
        this.controller = controller;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevatorPos = elevator.getPosition();
    }

    @Override
    public void execute() {
        if (controller.getPOV() == 0 && !elevator.isAtUpperLimit()) {
            elevatorPos -= 1.0;
        } else if (controller.getPOV() == 180 && !elevator.isAtLowerLimit()) {
            elevatorPos += 1.0;
        }

        elevator.moveToPosition(elevatorPos);

    }

}
