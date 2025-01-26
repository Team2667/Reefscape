// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.subsystems.Elevator;

import static frc.robot.Constants.AvailableSubsystems.*;
import static frc.robot.Constants.GameControllerConstants.*;

import java.security.cert.CertStoreException;

public class RobotContainer {
  private Elevator elevator;
  private XboxController driveTrainController;
  private XboxController manipulatorController;
  
  public RobotContainer() {
    // Initialize manipulatorController using manipulator gamepad Port constant
    initializeElevator(manipulatorController);
  }

  private void initializeElevator(XboxController controller){
    if (elevatorAvailable== true){
      initializeElevator(elevator);
      
      
      
    }
    // if the elevatorAvailable constant is set to true do the following
    // Initialize elevator.
    // Create an ElevatorDefaultCommand and make it the default command for the elevator subsystem.
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
