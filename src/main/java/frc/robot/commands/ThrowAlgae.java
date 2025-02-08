package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

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
