// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveCommand extends PIDCommand {
  DrivetrainSubsystem drivetrain;
  double output;
  /** Creates a new AutoMoveCommand. */
  public AutoMoveCommand(double distance, DrivetrainSubsystem drivetrain, double maxSpeed, double minSpeed) {
    super(
        // The controller that the command will use
        new PIDController(0.0005, 0, 0),
        // This should return the measurement
        () -> (drivetrain.getLeftDistance() + drivetrain.getRightDistance())/2,
        // This should return the setpoint (can also be a constant)
        () -> distance*DrivetrainConstants.TICKS_PER_INCH,
        // This uses the output
        output -> {
          SmartDashboard.putNumber("Speed", output);
          if(Math.abs(output) > maxSpeed) {
            output =  (output/Math.abs(output)) * maxSpeed;
          }
          if(Math.abs(output) < minSpeed) {
            output = (output/Math.abs(output)) * minSpeed;
          }
          drivetrain.arcadeDrive(output, 0);
        });
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        drivetrain.resetEncoders();
        this.getController().setTolerance(1500);
  }

  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    super.initialize();
  }

  @Override
  public void execute()  {
    SmartDashboard.putNumber("Error", this.getController().getPositionError());
    SmartDashboard.putNumber("Setpoint", this.getController().getSetpoint());
    SmartDashboard.putNumber("Left Encoder", drivetrain.getLeftDistance());
    SmartDashboard.putNumber("Right Encoder", drivetrain.getRightDistance());
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Is Finished", Math.abs(this.getController().getPositionError()) < this.getController().getPositionTolerance());
    return Math.abs(this.getController().getPositionError()) < this.getController().getPositionTolerance();
  }
}
