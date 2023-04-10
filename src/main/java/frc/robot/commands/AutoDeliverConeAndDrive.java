// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeliverConeAndDrive extends SequentialCommandGroup {
  /** Creates a new AutoDeliverConeAndDrive. */
  public AutoDeliverConeAndDrive(DrivetrainSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
    addCommands(
      new AutoDeliverConeTopCommand(elevator, arm, wrist, intake),
      //new WaitCommand(2),  //take out after qual 76
      new AutoMoveCommand(-150, drive, .6, 0.25)// untested
    );

    addRequirements();
  }
}
