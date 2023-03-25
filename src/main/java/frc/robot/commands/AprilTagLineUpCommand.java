// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AprilTagLineUpCommand extends CommandBase {
  /** Creates a new AprilTagLineUpcommand. */
  DrivetrainSubsystem drivetrain;
  LimelightSubsystem limelight;

  PIDController rotationController;
  PIDController speedController;

  double distance;

  public AprilTagLineUpCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, double distance) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.distance = distance;

    rotationController = new PIDController(0.0, 0.0, 0.0);
    speedController = new PIDController(0.0, 0.0, 0.0);

    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline(LimelightConstants.APRILTAG_PIPELINE);

    // TODO: add code for setpoint
    rotationController.setTolerance(0.0); // TODO: find optimal value

    speedController.setSetpoint(distance);
    speedController.setTolerance(0.0); // TODO: find optimal value
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.curvatureDrive(distance, distance, isFinished());
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
