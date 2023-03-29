// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAprilTagMoveCommand extends CommandBase {
  /** Creates a new AutoAprilTagMoveCommand. */
  private PIDController speedController;
  private PIDController rotationController;
  private DrivetrainSubsystem drivetrain;
  private LimelightSubsystem limelight;
  private double txTarget;
  private double tyTarget;
  private double maxSpeed = 0.6;
  private double minSpeed = 0.20;
  private int aprilTagID;
  private double[] limelightData = new double[6]; 

  public AutoAprilTagMoveCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, PIDController speedController, PIDController rotationController, double distance) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.speedController = speedController;
    this.rotationController = rotationController;
    this.txTarget = txTarget;
    this.tyTarget = tyTarget;
    this.aprilTagID = aprilTagID;

    NetworkTableInstance.getDefault().getTable("Limelight").getEntry("targetpos_robotspace").getDoubleArray(new double[6]);
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline(LimelightConstants.APRILTAG_PIPELINE);
    speedController.setSetpoint(tyTarget*DrivetrainConstants.TICKS_PER_INCH);
    rotationController.setSetpoint(txTarget);
    speedController.setTolerance(15000);
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedController.calculate((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2.0 );
     double rotation = rotationController.calculate(tyTarget); 

     if(Math.abs(speed) > maxSpeed){
      speed = Math.abs((speed)/speed) * maxSpeed;
     }
     if(Math.abs(speed) < minSpeed){
      speed = Math.abs((speed)/speed) * minSpeed;
     }

     drivetrain.curvatureDrive(speed, rotation, true );
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
