// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTopLevelCommand extends SequentialCommandGroup {
  /** Creates a new SetLevelCommand. */
  public SetTopLevelCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SetArmExtensionCommand(0, arm),  
    new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
    new SetArmExtensionCommand(ArmConstants.TOP_EXTENSION, arm),
    new LockElevatorCommand(new SetPointSupplier(), elevator)
    );
    addRequirements(elevator, arm);
  }
}
