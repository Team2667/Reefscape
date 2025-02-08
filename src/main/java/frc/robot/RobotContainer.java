// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.PickAlgae;
import frc.robot.commands.ThrowAlgae;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

import static frc.robot.Constants.AvailableSubsystems.*;
import static frc.robot.Constants.GameControllerConstants.*;

public class RobotContainer {
  private Elevator elevator;
  private Claw claw;
  private Arm arm;
  private CommandXboxController driveTrainController;
  private CommandXboxController manipulatorController;
  
  public RobotContainer() {
    // Initialize manipulatorController using manipulator gamepad Port constant
    manipulatorController = new CommandXboxController(manipulatorGamepadPort);
    initializeElevator(manipulatorController);
    initializeClaw(manipulatorController);
    initializeArm(manipulatorController);
  }

  private void initializeElevator(CommandXboxController controller){
    if (elevatorAvailable== true){      
       elevator = new Elevator();

      Command elevatorC = new ElevatorDefaultCommand(elevator, controller.getHID());
      elevator.setDefaultCommand(elevatorC);
  
    }
  }

  private void initializeClaw(CommandXboxController controller){
    if (clawAvailable == true){
      claw = new Claw();
      Command clawIntake = new PickAlgae(claw);
      controller.leftBumper().whileTrue(clawIntake);
      Command clawThrow = new ThrowAlgae(claw);
      controller.rightBumper().whileTrue(clawThrow);
      }
  }

  private void initializeArm(CommandXboxController controller) {
    if (armAvailavble == true) {
      arm = new Arm();
      Command armOut = new ArmDefaultCommand(arm, controller.getHID());
      arm.setDefaultCommand(armOut);
    }
    // and create a ArmDefaultCommand
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
