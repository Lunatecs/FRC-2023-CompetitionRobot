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
public class AutoChargingStation extends SequentialCommandGroup {
  /** Creates a new AutoChargingStation. */

  public AutoChargingStation(DrivetrainSubsystem drivetrain) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoMoveCommand(-146, drivetrain, .6, 0.25),
      new WaitCommand(0.25),
      new AutoMoveCommand(59.5, drivetrain, .45, 0.25), //was 57//was 69
      new AutoBalanceCommand(drivetrain)
    );
  }
}
