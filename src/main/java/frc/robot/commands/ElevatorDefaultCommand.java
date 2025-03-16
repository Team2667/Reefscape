package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDefaultCommand  extends Command{
    private XboxController controller;
    private Elevator elevator;
    private double elevatorPos;

    public ElevatorDefaultCommand(Elevator elevator, XboxController controller ) {
        this.controller = controller;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    // TODO: PL1 - Add an initialize method. The intialize method will set elevatorPos to the position of the elevator.
    @Override
    public void initialize() {
        elevatorPos = elevator.getPosition();
    }

    @Override
    public void execute() {

        // TODO: PL01 - implement this method as follows:
        // if controller.getPOV() == 0 and elevator is not at the upper limit, substract 1.0 from elevatorPOS.
        // if controller.getPOV() == 180 and elevator is not at the lower limit, add 1.0 to elevatorPOS;
        // call moveToPosition on the elevator and pass it elevatorPos
    }   
    
    // TODO: PL01 - Remove this method
    public void end(boolean isInturupted) {
        elevator.stop();
    }
}
