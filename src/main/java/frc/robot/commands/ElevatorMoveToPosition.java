package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class ElevatorMoveToPosition extends Command{
    private Elevator elevator;
    private ElevatorPosition targetPosition;
    private ElevatorFeedforward ff;
    
    public ElevatorMoveToPosition(Elevator elevator, ElevatorPosition targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        this.ff = new ElevatorFeedforward(1.2, 0.09, 19.0);
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.moveToPosition(targetPosition.position, ff.calculateWithVelocities(elevator.getVelocity(), 0.0));
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtPosition(targetPosition);
    }

    @Override
    public void end(boolean isInteruppted) {
    }
}
