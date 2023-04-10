// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoScoochBalanceCommand extends CommandBase {
  /** Creates a new AutoScoochBalanceCommand. */
  DrivetrainSubsystem drivetrain;
  long time;
  double zeroAngle;
  PIDController balance;
  public AutoScoochBalanceCommand(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    balance = new PIDController(0.11, 0, 0);
    zeroAngle = drivetrain.getZeroAngle();
    balance.setSetpoint(zeroAngle);
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = new Date().getTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long now = new Date().getTime();
    if (now >= time + 1250) {
      time = new Date().getTime();
      drivetrain.arcadeDrive(0, 0);
    } else if (now >= time + 750) {
      drivetrain.arcadeDrive(0, 0);
    } else {
      drivetrain.arcadeDrive(balance.calculate(drivetrain.getPitch()), 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
