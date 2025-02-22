package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class ElevatorMoveToPosition extends Command{
    private Elevator elevator;
    private ElevatorPosition targetPosition;
    
    public ElevatorMoveToPosition(Elevator elevator, ElevatorPosition targetPosition) {
        // TODO: Initialize the class member values
        // TODO: Add elevator as a requirement for this command
    }

    // TODO: Add an initialize method that moves the elevator to targetPosition
    // See PickAlgae for an example.

    // TODO: isFinished should return true if the elevator is at the target position.
    @Override
    public boolean isFinished() {
        return false;
    }

    // TODO: implement the End method. See PickAlgae for an example.
}
