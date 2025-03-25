package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberForward extends Command {
    private Climber climber;

    public ClimberForward(Climber climber) {
        this.climber = climber;
        this.addRequirements(climber);
    }

    public void intiialize() {
        climber.forward();
    }

    public void end(Boolean isFinished) {
        climber.stop();
    }
}
