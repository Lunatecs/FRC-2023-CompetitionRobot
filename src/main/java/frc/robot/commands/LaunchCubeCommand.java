// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchCubeCommand extends SequentialCommandGroup {
  /** Creates a new LaunchCubeCommand. */
  public LaunchCubeCommand(WristSubsystem wrist, IntakeSubsystem intake, ElevatorSubsystem elevator, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MID_HEIGHT, 0.00006),
      new FlickWristCommand(wrist, intake),
      new SetOtherLevelsCommand(elevator, arm, wrist, ElevatorConstants.BOTTOM, WristConstants.WRIST_HOME, 0.00004)
    );
  }
}
