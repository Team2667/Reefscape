package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class PickAlgae extends Command {

    private Claw claw;

    public PickAlgae(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }
 
    @Override
    public void initialize() {
        claw.pickAlgae();
    }

    // TODO: Implement this command by calling a method on the claw subsystem.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isFinished) {
        claw.stop();
    }
}
