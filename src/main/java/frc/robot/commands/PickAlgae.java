package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class PickAlgae extends Command {
    // Create a command that will suck algae off the and into the claw.
    // 
    // TODO: Declare a member variable for the claw subsystem

    // TODO: Create a constructor method that has one parameter for the Claw subsystem.
    // It will initialize the member variable that you delcared in the previous step and
    // Add claw to its required list. Hint: See the constructor in the ElevatorDefaultCommand
    // class.
    //


    @Override
    public void initialize() {
        // TODO: Write code to move the motors in a direction that will bring in the algae
    }

    @Override
    public void end(boolean isFinished) {
        // TODO: Write code that will stop the motor
    }

    // TODO: Write a simmilar class called ThrowAlgae to this one that will throw the algae.
}
