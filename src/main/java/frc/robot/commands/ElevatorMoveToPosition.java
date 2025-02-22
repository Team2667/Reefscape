package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class ElevatorMoveToPosition extends Command{
    private Elevator elevator;
    private ElevatorPosition targetPosition;
    
    public ElevatorMoveToPosition(Elevator elevator, ElevatorPosition targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }
    public void initialize(Elevator elevator, ElevatorPosition targetPosition) {
        elevator.moveToPosition(targetPosition);
    }
    @Override
    public boolean isFinished() {
        return elevator.isAtPosition(targetPosition);
    }
    public void end(Elevator elevator) {
        elevator.stop();
    }
}
