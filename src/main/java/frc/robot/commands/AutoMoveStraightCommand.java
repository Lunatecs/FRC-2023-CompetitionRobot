// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.SetPointSupplier;

public class AutoMoveStraightCommand extends CommandBase {
  /** Creates a new AutoMoveStraightCommand. */
  private PIDController speedController;
  private PIDController rotationController;
  private DrivetrainSubsystem drivetrain;
  private double distance;
  private double maxSpeed = 0.6;
  private double minSpeed = 0.20;

  public AutoMoveStraightCommand(DrivetrainSubsystem drivetrain, PIDController speedController, PIDController rotationController, double distance) {
    this.drivetrain = drivetrain;
    this.rotationController = rotationController;
    this.speedController = speedController;
    this.distance = distance;
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedController.setSetpoint(distance*DrivetrainConstants.TICKS_PER_INCH);
    rotationController.setSetpoint(drivetrain.getYaw());
    speedController.setTolerance(1500);
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    double speed = speedController.calculate((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2.0);
    double rotation = rotationController.calculate(drivetrain.getYaw());
    SmartDashboard.putNumber("Straight Auto Raw Speed", speed);
    
    if(Math.abs(speed) > maxSpeed) {
      speed =  (Math.abs(speed)/speed) * maxSpeed;
    }
    if(Math.abs(speed) < minSpeed) {
      speed = (Math.abs(speed)/speed) * minSpeed;
    }
    
    SmartDashboard.putNumber("Straight Auto Distance Error", speedController.getPositionError());
    SmartDashboard.putNumber("Straight Auto Speed Output", speed);
    SmartDashboard.putNumber("Straight Auto Rotation Error", rotationController.getPositionError());
    SmartDashboard.putNumber("Straight Auto Rotation Output", rotation);



    drivetrain.curvatureDrive(speed*-1.0, rotation*-1.0, false);
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
