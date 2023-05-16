// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceStuff extends SequentialCommandGroup {
  /** Creates a new BalanceStuff. */
  public BalanceStuff(DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoMoveCommand(-146, drivetrain, .65, 0.25),
      new WaitCommand(2), //was 0.5
      new AutoMoveCommand(62, drivetrain, .55, 0.25),
      new AutoForwardUntilChangeInGyro(drivetrain),
      //new AutoBalanceSlowCommand(drivetrain)
      new AutoReverseScootchCommand(drivetrain)
    );
  }
}
