// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAprilTagBotposeMoveCommand extends CommandBase {
  private PIDController speedController;
  private PIDController rotationController;
  private DrivetrainSubsystem drivetrain;
  private LimelightSubsystem limelight;
  private double xTarget;
  private double yTarget;
  private double maxSpeed = 0.6;
  private double minSpeed = 0.20;
  private int aprilTagID;

  public AutoAprilTagBotposeMoveCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, PIDController speedController, PIDController rotationController, double xTarget, double yTarget) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.speedController = speedController;
    this.rotationController = rotationController;
    this.xTarget = xTarget;

    
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
