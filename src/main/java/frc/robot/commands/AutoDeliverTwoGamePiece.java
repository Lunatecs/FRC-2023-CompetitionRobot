// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import frc.robot.subsystems.WristSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeliverTwoGamePiece extends SequentialCommandGroup {
  /** Creates a new AutoDeliverTwoGamePiece. */
  public AutoDeliverTwoGamePiece(ElevatorSubsystem elevator, IntakeSubsystem intake, DrivetrainSubsystem drive, ArmSubsystem arm, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> intake.runIntake(-.15), intake),
      new SetWristAngleCommand(wrist, WristConstants.WRIST_HOME),
      new SetArmExtensionCommand(0, arm),
      new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
      new SetArmExtensionCommand(ArmConstants.MAX_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.CONE_SETPOINT),
      new InstantCommand(() -> intake.runIntake(0.75), intake),
      new WaitCommand(0.5),
      new InstantCommand(() -> intake.runIntake(0), intake),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0 , WristConstants.WRIST_HOME, true),
      new AutoMoveCommand(-174, drive, .6, .25),
      new AutoTurnCommand(drive, 155),
      new SetArmExtensionCommand(ArmConstants.MAX_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.GROUND_INTAKE_CUBE),
      new InstantCommand(() -> intake.runIntake(-.75), intake),
      new WaitCommand(0.4),
      new InstantCommand(() -> intake.runIntake(-.15)),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0, WristConstants.WRIST_HOME, true),
      new AutoTurnCommand(drive, -155),
      new AutoMoveCommand(174, drive, 0.6, 0.25),
      new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
      new SetArmExtensionCommand(ArmConstants.MAX_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.CONE_SETPOINT),
      new InstantCommand(() -> intake.runIntake(0.75), intake),
      new WaitCommand(0.5),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0 , WristConstants.WRIST_HOME, true),
      new AutoTurnCommand(drive, 5)
   
      
      
    );
  }
}
