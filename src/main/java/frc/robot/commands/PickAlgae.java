package frc.robot.commands;
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

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isFinished) {
        claw.stop();
    }
}
