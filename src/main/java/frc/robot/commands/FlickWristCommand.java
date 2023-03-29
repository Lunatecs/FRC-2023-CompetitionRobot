// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class FlickWristCommand extends CommandBase {
  private WristSubsystem wrist;
  private IntakeSubsystem intake;
  private boolean finished;
  /** Creates a new FlickWristCommand. */
  public FlickWristCommand(WristSubsystem wrist, IntakeSubsystem intake) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.turnWrist(-0.5);
    if (wrist.getWristEncoder() < WristConstants.LAUNCH_CUBE) {
      intake.runIntake(1.0);
    }
    if (wrist.getWristEncoder() < WristConstants.STOP_LAUNCH) {
      wrist.turnWrist(0);
      intake.runIntake(0);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.turnWrist(0);
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
