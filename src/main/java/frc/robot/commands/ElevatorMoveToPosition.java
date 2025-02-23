package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class ElevatorMoveToPosition extends Command{
    // The following 2 variables can be accessed by any of the methods in this class.   
    private Elevator elevator;
    private ElevatorPosition targetPosition;
    
    public ElevatorMoveToPosition(Elevator elevator, ElevatorPosition targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    // TODO: Remove all parameters and add a @Override annotation above this method like
    // isFinished. The elevator and targetPosition variables are already class member variables. 
    public void initialize(Elevator elevator, ElevatorPosition targetPosition) {
        elevator.moveToPosition(targetPosition);
    }


    @Override  // <-- Override annotation
    public boolean isFinished() {
        return elevator.isAtPosition(targetPosition);
    }

    //TODO: Replace the Elevator parameter with a boolean parameter. The name of the boolean parameter does not matter
    // and the parameter itself will not be used but it still needs to be there. 
    // Add the Override annotation to this method.
    public void end(Elevator elevator) {
        elevator.stop();
    }
}
