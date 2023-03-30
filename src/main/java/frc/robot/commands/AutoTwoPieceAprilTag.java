// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoPieceAprilTag extends SequentialCommandGroup {
  /** Creates a new AutoTwoPieceAprilTag. */
  public AutoTwoPieceAprilTag(ElevatorSubsystem elevator, IntakeSubsystem intake, DrivetrainSubsystem drive, ArmSubsystem arm, WristSubsystem wrist, LimelightSubsystem limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDeliverConeTopCommand(elevator, arm, wrist, intake),
      //new AutoMoveCommand(-174, drive, .6, .25),
      new AutoMoveStraightCommand(drive, new PIDController(0.0005, 0.0, 0.0), new PIDController(0.05, 0.0, 0.0), -158),
      new AutoTurnCommand(drive, -145, 0.05),
      new InstantCommand(() -> intake.runIntake(-.15)),
     // new SetArmExtensionCommand(ArmConstants.MAX_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.GROUND_INTAKE_CUBE),
      new SetArmExtensionCommand(ArmConstants.GROUND_EXTENSION, arm),
      new InstantCommand(() -> intake.runIntake(-.75), intake),
      new WaitCommand(0.6),
      //new SetArmExtensionCommand(0, arm),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0, WristConstants.WRIST_HOME, true),
      new InstantCommand(() -> intake.runIntake(-.3)),
      new AutoTurnCommand(drive, 150, 0.05),
      //new AutoMoveCommand(174, drive, 0.6, 0.25),

      new AutoAprilTagMoveCommand(drive, limelight, new PIDController(0.0005, 0.0, 0.0), new PIDController(0.05, 0.0, 0.0), 0, 156, 3)
      /* 
      new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
      new SetArmExtensionCommand(ArmConstants.MAX_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.CONE_SETPOINT),
      new InstantCommand(() -> intake.runIntake(0.30), intake),
      new WaitCommand(0.5),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0 , WristConstants.WRIST_HOME, true),
      new InstantCommand(() -> intake.runIntake(0)),
      new AutoTurnCommand(drive, 5, 0.05)
      /*
      new AutoMoveStraightCommand(drive, new PIDController(0.0005, 0.0, 0.0), new PIDController(0.05, 0.0, 0.0), 156),
      new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
      new SetArmExtensionCommand(ArmConstants.MAX_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.CONE_SETPOINT),
      new InstantCommand(() -> intake.runIntake(0.30), intake),
      new WaitCommand(0.5),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0 , WristConstants.WRIST_HOME, true),
      new InstantCommand(() -> intake.runIntake(0)),
      new AutoTurnCommand(drive, 5, 0.05)
      */
    );
  }
}
