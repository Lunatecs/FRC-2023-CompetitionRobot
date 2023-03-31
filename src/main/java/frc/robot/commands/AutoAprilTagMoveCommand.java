// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.LimelightHelpers;

public class AutoAprilTagMoveCommand extends CommandBase {
  /** Creates a new AutoAprilTagMoveCommand. */
  private PIDController speedController;
  private PIDController rotationController;
  private DrivetrainSubsystem drivetrain;
  private LimelightSubsystem limelight;
  private double txTarget;
  private double distance;
  private double maxSpeed = 0.6;
  private double minSpeed = 0.20;
  private int pipeline;

  public AutoAprilTagMoveCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, PIDController speedController, PIDController rotationController, double txTarget, double distance, int pipeline) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.speedController = speedController;
    this.rotationController = rotationController;
    this.txTarget = txTarget;
    this.distance = distance;
    this.pipeline = pipeline;

    
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline(pipeline);
    speedController.setSetpoint(distance*DrivetrainConstants.TICKS_PER_INCH);
    rotationController.setSetpoint(txTarget);
    speedController.setTolerance(1500);
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedController.calculate(limelight.getTZ()*DrivetrainConstants.METERS_TO_INCHES*DrivetrainConstants.TICKS_PER_INCH);
    double rotation = rotationController.calculate(limelight.getTX()); 

    if(Math.abs(speed) > maxSpeed){
      speed = Math.abs((speed)/speed) * maxSpeed;
    }
      
    if(Math.abs(speed) < minSpeed){
      speed = Math.abs((speed)/speed) * minSpeed;
    }

    drivetrain.curvatureDrive(speed, rotation, false);

    SmartDashboard.putNumber("April Tag Auto Distance Error", speedController.getPositionError());
    SmartDashboard.putNumber("April Tag Auto Speed Output", speed);
    SmartDashboard.putNumber("April Tag Auto Rotation Error", rotationController.getPositionError());
    SmartDashboard.putNumber("April Tag Auto Rotation Output", rotation);
  }

     

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speedController.atSetpoint();
  }
}
