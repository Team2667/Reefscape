package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberReverse extends Command{

    private Climber climber;

    public ClimberReverse(Climber climber){
        this.climber = climber;
        this.addRequirements(climber);
    } 

    public void intiialize() {
        climber.reverse();
    } 
    public void end(Boolean isFinished) {
        climber.stop();
    }
}
