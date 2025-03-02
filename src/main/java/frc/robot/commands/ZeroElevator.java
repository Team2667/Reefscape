package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command{
    public Elevator elevator;

    @Override
    public void initialize() {
        elevator.moveDown();
    }
    @Override
    public boolean isFinished() {
        if (elevator.isAtLowerLimit()) {
            return true;
        }
        else {
            return false;
        }
    }
    @Override
    public void end(boolean isInteruppted) {
        elevator.ZeroElevator();
    }

    // Background:
    // The elevator uses a realitive encoder to determine the position of the elevator (how high or low it is).
    // The position reading from a relative encoder is 0 when the robot starts up. The reading changes from that initial position 
    // as the elevator is raised and lowered. If the elevator is midway between its highest and lowest positions when the 
    // robot starts up, then that will be its 0 position. We need the 0 position to be the elevators lowest position.
    // We can do this by lowering the elvator to its lowest position setting the encoders position value to 0. This is something
    // we will do first thing in autonomous mode.
    //
    // TODO: 
    // * Before working on this class, make sure you have corrected the issues with ElevatorMoveToPosition
    // * Implement the initialize method. The initialize method should cause the elvator to move down.
    // * Implement the isFinished method. The isFinished method should return true if the elevator is at its lowest
    //   limit and false otherwise.
    // * Implement the end method. The end method should zero the elevator.
    // * The signiture for the initiailze, isFinished and end methods should be the same as ElevatorMoveToPosition 
    //   after that class has been corrected.
    //
}
