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
        // TODO: PL03 initalize armPosition to the arms current position.
    }

    @Override
    public void execute() {
        // TODO: PLO3 if controller POV is 90 and arm is not at the forward limit
        // Set the state member variable to to the next state clockwise.

        // if controller POV is 270 and arm is not at the reverse limit,
        // Set the state member variable to the next state counter clockwise.

        // call arm.runToPosition

        if (controller.getPOV() == 90)
        arm.runToPosition(state);
    }

    private TrapezoidProfile.State getNextStateClockwise() {
        return new TrapezoidProfile.State(state.position += stepAmount, 0.0);
    }

    private TrapezoidProfile.State getNextCopunterStateClockwise() {
        return new TrapezoidProfile.State(state.position -= stepAmount, 0.0);
    }
}
