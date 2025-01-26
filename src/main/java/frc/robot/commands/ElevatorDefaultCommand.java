package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDefaultCommand  extends Command{
    private XboxController controller;
    private Elevator elevator;

    public ElevatorDefaultCommand(Elevator elevator, XboxController controller ) {
        this.controller = controller;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if(controller.getPOV()== 0){
            elevator.moveUp();
        }
        else if(controller.getPOV()== 180) {
            elevator.moveDown();
        }
        else {
            elevator.stop();
        }
    
    }   
public void end(boolean isInturupted) {
    elevator.stop();
}

}
