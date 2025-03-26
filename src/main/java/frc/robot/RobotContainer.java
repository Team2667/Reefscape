// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmMoveToPosition;
import frc.robot.commands.ClimberForward;
import frc.robot.commands.ClimberReverse;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveTrainAddThrottle;
import frc.robot.commands.DriveTrainRemoveThrottle;
import frc.robot.commands.DriveTrainZeroIMU;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.ElevatorMoveToPosition;
import frc.robot.commands.PickAlgae;
import frc.robot.commands.ThrowAlgae;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

import static frc.robot.Constants.AvailableSubsystems.*;
import static frc.robot.Constants.GameControllerConstants.*;
import static frc.robot.Constants.PoseEstimatorVals.usePhotonVision;

import java.lang.ModuleLayer.Controller;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;

public class RobotContainer {
  private Elevator elevator;
  private Claw claw;
  private Arm arm;
  private DriveTrain driveTrain;
  private Climber climber;
  private PoseEstimatorSubsystem poseEstimator;
  private CommandXboxController driveTrainController;
  private CommandXboxController manipulatorController;
  private TrajectoryCommandFactory trajectoryCommandFactory;
  private SendableChooser<Command> sendableChooser;
  
  public RobotContainer() {
    // Initialize manipulatorController using manipulator gamepad Port constant
    manipulatorController = new CommandXboxController(manipulatorGamepadPort);
    driveTrainController = new CommandXboxController(driveTrainGamepadPort);
    sendableChooser = new SendableChooser<>();
    initializeDriveTrain(driveTrainController);
    initializeElevator(manipulatorController);
    initializeClaw(manipulatorController);
    initializeArm(manipulatorController);
    initializePoseEstimator(driveTrain);
    initializeManipulatorCompoundCommands(manipulatorController);
    initializeClimber(driveTrainController);
    initializeAutonomousCommands();

  }

  private void initializeElevator(CommandXboxController controller){
    if (elevatorAvailable== true){      
       elevator = new Elevator();     
      Command zeroElevator = new ZeroElevator(elevator);
      controller.back().onTrue(zeroElevator);
    }
  }
  

  private void initializeClaw(CommandXboxController controller){
    if (clawAvailable == true){
      claw = new Claw();
      Command clawIntake = new PickAlgae(claw);
      controller.leftBumper().onTrue(clawIntake);
      Command clawThrow = new ThrowAlgae(claw);
      controller.rightBumper().whileTrue(clawThrow);
      }
  }

  private void initializeArm(CommandXboxController controller) {
    if (armAvailavble == true) {
      // TODO: create a command to move the arm to a specific position and bind it to the A button.
      arm = new Arm();
      Command armOut = new ArmDefaultCommand(arm, controller.getHID());
    //  arm.setDefaultCommand(armOut);

    }
  }

  private void initializeDriveTrain(CommandXboxController controller) {
    if (driveTrainAvailable) {
      driveTrain = new DriveTrain();
      driveTrain.setDefaultCommand(new DefaultDriveCommand(driveTrain,
          () -> -modifyAxis(controller.getHID().getLeftY()),
          () -> -modifyAxis(controller.getHID().getLeftX()),
          () -> modifyAxis(controller.getHID().getRightX())));

      controller.rightTrigger(.5).onTrue(new DriveTrainAddThrottle(driveTrain));
      controller.rightTrigger(.5).onFalse(new DriveTrainRemoveThrottle(driveTrain));
      controller.leftStick().onTrue(new DriveTrainZeroIMU(driveTrain));
    }
  }

  private void initializePoseEstimator(DriveTrain driveTrain) {
    if (poseEstimatorAvailable && driveTrainAvailable) {
       var photonCamera = new PhotonCamera("USB_webcam");
      poseEstimator = new PoseEstimatorSubsystem(photonCamera, driveTrain);
    }
  }
  // change the "CMD#" to more descriptive names
  private void initializeManipulatorCompoundCommands(CommandXboxController controller) {
    if (driveTrainAvailable && armAvailavble) {
      ElevatorMoveToPosition CMD1 = new ElevatorMoveToPosition(elevator, ElevatorPosition.LowerRefPosition);
      ArmMoveToPosition CMD2 = new ArmMoveToPosition(arm, ArmPosition.LowReef);
      controller.x().onTrue(CMD1.andThen(CMD2));
      
      ElevatorMoveToPosition CMD3 = new ElevatorMoveToPosition(elevator, ElevatorPosition.HomePosition);
      ArmMoveToPosition CMD4 = new ArmMoveToPosition(arm, ArmPosition.Home);
      controller.leftStick().onTrue(CMD4.andThen(CMD3));
      
      ElevatorMoveToPosition CMD5 = new ElevatorMoveToPosition(elevator, ElevatorPosition.TopRefPosition);
      ArmMoveToPosition CMD6 = new ArmMoveToPosition(arm, ArmPosition.HighReef);
      controller.y().onTrue(CMD5.andThen(CMD6));

      ElevatorMoveToPosition CMD7 = new ElevatorMoveToPosition(elevator, ElevatorPosition.offGroundPos);
      ArmMoveToPosition CMD8 = new ArmMoveToPosition(arm, ArmPosition.offGround);
      controller.rightStick().onTrue(CMD7.andThen(CMD8));

      ElevatorMoveToPosition CMD9 = new ElevatorMoveToPosition(elevator, ElevatorPosition.offCoralPos);
      ArmMoveToPosition CMD10 = new ArmMoveToPosition(arm, ArmPosition.OffCoral);
      controller.a().onTrue(CMD9.andThen(CMD10));

      controller.start().whileTrue(new ElevatorDefaultCommand(elevator, controller.getHID()));

      ElevatorMoveToPosition ElevatorToBarge = new ElevatorMoveToPosition(elevator, ElevatorPosition.toBargePos);
      ArmMoveToPosition ArmToBarge = new ArmMoveToPosition(arm, ArmPosition.ToBarge);
      controller.b().onTrue(ElevatorToBarge.andThen(ArmToBarge));

      //TODO: PL04 - Add a command for positioning the robot to put the algae in the barge

      //TODO: PL05 - Use alongWith instead of andThen to combine the commands. This should cause the
      // compound commands to run syncronously instead of in series.
    }
  }

  private void initializeClimber(CommandXboxController controller) {
    if (climberAvailable) {
      climber = new Climber();
      ClimberForward climberForward = new ClimberForward(climber);
      controller.rightBumper().whileTrue(climberForward);

      ClimberReverse climberReverse = new ClimberReverse(climber);
      controller.leftBumper().whileTrue(climberReverse);
    }
  }

  private void initializeAutonomousCommands() {
    if (poseEstimatorAvailable && driveTrainAvailable) {
      sendableChooser.addOption("Score from middle No April Tags", createScoreFromMiddleAutonomousNoAprilTags());
      if (usePhotonVision) {
        sendableChooser.addOption("Blue Score from middle April Tags", createBlueScoreFromMiddleAutonomous());
      }
    }
  }

  private Command createScoreFromMiddleAutonomousNoAprilTags() {
    Transform2d transformStart = new Transform2d(2.2, 0.0, new Rotation2d(0));
    trajectoryCommandFactory = new TrajectoryCommandFactory(driveTrain, poseEstimator);
    Pose2d startingPos = poseEstimator.getPosition();
    Pose2d endingPos = startingPos.transformBy(transformStart);
    Command moveCommand = trajectoryCommandFactory.createTrajectoryCommand(startingPos, new ArrayList<Translation2d>(), endingPos);
    return moveCommand;
  }

  private Command createBlueScoreFromMiddleAutonomous() {
    trajectoryCommandFactory = new TrajectoryCommandFactory(driveTrain, poseEstimator);
    Pose2d startingPos = poseEstimator.getPosition();
    Pose2d endingPos = new Pose2d(5.2, 4.0259, Rotation2d.k180deg);
    Command moveCommand = trajectoryCommandFactory.createTrajectoryCommand(startingPos, new ArrayList<Translation2d>(), endingPos);
    return moveCommand;
  }

  private Command createBlueScoreFromLeft() {
    Transform2d transformStart = new Transform2d(2.2, 0.0, new Rotation2d(0));
    trajectoryCommandFactory = new TrajectoryCommandFactory(driveTrain, poseEstimator);
    Pose2d startingPos = poseEstimator.getPosition();
    Pose2d endingPos = startingPos.transformBy(transformStart);
    Command moveCommand = trajectoryCommandFactory.createTrajectoryCommand(startingPos, new ArrayList<Translation2d>(), endingPos);
    return moveCommand;
  }

  public Command getAutonomousCommand() {
    if (sendableChooser != null) {
      return sendableChooser.getSelected();
    } else {
      return Commands.print("No autonomous command configured");
    }
  }

  private double modifyAxis(double input) {
    input = deadband(input, 0.05);
    // Square the axis
    return Math.copySign(input * input, input);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
