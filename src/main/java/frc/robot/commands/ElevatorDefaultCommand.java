package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDefaultCommand  extends Command{
    private XboxController controller;
    private Elevator elevator;

    public ElevatorDefaultCommand(Elevator elevator, XboxController controller ) {
        // intialize this.controller and this.elevator
        // add elevator as a requirement for this command
    }

    @Override
    public void execute() {
        // If the up arrow on the dpad is pressed, go up.
        // If the down arrow on the dpad is pressed, go down.
        // See https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/GenericHID.html#getPOV()
    }
}
