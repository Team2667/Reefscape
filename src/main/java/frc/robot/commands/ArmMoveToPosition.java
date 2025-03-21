package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmMoveToPosition extends Command {
  private Arm arm;
  private Arm.ArmPosition setpoint;
  private TrapezoidProfile.State state;
  /** Creates a new RunArmClosedLoop. */
  public ArmMoveToPosition(Arm ar, Arm.ArmPosition target) {
    arm = ar;
    setpoint = target;
    state = new State(setpoint.position, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  @Override
  public void initialize() {
    System.out.println("Arm Move to Position started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.runToPosition(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm Move to Position ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtSetPoint(setpoint);
  }
}
