// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeliverConeCommunityBalance extends SequentialCommandGroup {
  /** Creates a new AutoDeliverConeCommunityBalance. */
  public AutoDeliverConeCommunityBalance(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new AutoDeliverConeTopCommand(elevator, arm, wrist, intake),
      new AutoMoveCommand(-146, drivetrain, .6, 0.25),
      new AutoTurnCommand(drivetrain, -176, 0.05),
      new AutoMoveCommand(-59.5, drivetrain, .45, 0.25),
      new AutoForwardUntilChangeInGyro(drivetrain),
      new WaitCommand(.25),
      //new AutoBalanceSlowCommand(drivetrain)
      new AutoScoochBalanceCommand(drivetrain)
    );
  }
}
