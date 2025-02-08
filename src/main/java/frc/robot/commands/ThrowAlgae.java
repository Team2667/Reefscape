package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class PickAlgae extends Command {
    public Claw claw;
    // Create a command that will suck algae off the and into the claw.
    // 
    public PickAlgae(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }


    // TODO: Create a constructor method that has one parameter for the Claw subsystem.
    // It will initialize the member variable that you delcared in the previous step and
    // Add claw to its required list. Hint: See the constructor in the ElevatorDefaultCommand
    // class.
    //


    @Override
    public void initialize() {
        
            claw.pickAlgae();
    }


    @Override
    public void end(boolean isFinished) {
        // TODO: Write code that will stop the motor
        claw.stop();
    }

    // TODO: Write a simmilar class called ThrowAlgae to this one that will throw the algae.
}

public class ThrowAlgae extends Command {
    public Claw claw;
    public ThrowAlgae(Claw claw) {
        this.claw = claw;
        addRequirements(claw);

    }
    @Override
    public void initialize() {
        claw.throwAlgae();
    }
    @Override
    public void end(boolean isFinished) {
        claw.stop();
    }

}
