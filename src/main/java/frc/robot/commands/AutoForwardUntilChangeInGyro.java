// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardUntilChangeInGyro extends CommandBase {
  DrivetrainSubsystem drivetrain;
  double gyroSetpoint = 0.0;
  boolean isFinished = false;

  public AutoForwardUntilChangeInGyro(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroSetpoint = drivetrain.getPitch();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Move Gyro set point", gyroSetpoint);
    SmartDashboard.putNumber("Auto Move Gyro -1.25", Math.abs(gyroSetpoint) - 1.75);
    SmartDashboard.putNumber("Auto Move Gyro Pitch", drivetrain.getPitch());
    SmartDashboard.putBoolean("Auto Move Gyro Should I move?", Math.abs(gyroSetpoint) - 1.0 < drivetrain.getPitch() && !isFinished);
    SmartDashboard.putBoolean("Auto Move Gyro isFinished", isFinished);
    SmartDashboard.putNumber("Auto Move Gyro finding 5", Math.abs(Math.abs(drivetrain.getZeroAngle()) - Math.abs(drivetrain.getPitch())));

    if(gyroSetpoint < drivetrain.getPitch()) {
      gyroSetpoint = drivetrain.getPitch();
    }
    if(Math.abs(Math.abs(drivetrain.getZeroAngle()) - Math.abs(drivetrain.getPitch())) > 4.0 && !isFinished) {
    //if(Math.abs(gyroSetpoint) - 1.25 < drivetrain.getPitch() && !isFinished) {
      drivetrain.curvatureDrive(0.2, 0.0,false);
    } else {
      drivetrain.curvatureDrive(0.0, 0.0,false);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.curvatureDrive(0.0, 0.0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
