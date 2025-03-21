package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDefaultCommand extends Command{
    private XboxController controller;
    private Arm arm;
    private TrapezoidProfile.State state;
    private static double stepAmount = .01;

    public ArmDefaultCommand(Arm arm, XboxController controller) {
        this.controller = controller;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        state = new TrapezoidProfile.State(arm.getArmPosition(), 0.0);
    }

    @Override
    public void execute() {
        if (controller.getPOV() == 90 ){
            state = getNextStateClockwise();
        } else if (controller.getPOV() == 270) {
            state = getNextCopunterStateClockwise();
        }
        arm.runToPosition(state);
    }

    private TrapezoidProfile.State getNextStateClockwise() {
        return new TrapezoidProfile.State(state.position += stepAmount, 0.0);
    }

    private TrapezoidProfile.State getNextCopunterStateClockwise() {
        return new TrapezoidProfile.State(state.position -= stepAmount, 0.0);
    }
}
