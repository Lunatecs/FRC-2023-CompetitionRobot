// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceSlowCommand extends PIDCommand {
  /** Creates a new AutoBalanceCommand. */

  private DrivetrainSubsystem drivetrain;

  public AutoBalanceSlowCommand(DrivetrainSubsystem drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(0.17, 0.0165, 0.0),  //p=0.165
        // This should return the measurement
        () -> drivetrain.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> drivetrain.getZeroAngle(),
        // This uses the output
        output -> {
          // Use the output here
          SmartDashboard.putNumber("Auto Slow output", output);
          if(Math.abs(output) > 0.325) {
            output = Math.signum(output) * 0.325;
          }
          SmartDashboard.putNumber("Auto Slow After output", output);
          drivetrain.arcadeDrive(-output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    this.getController().setTolerance(1);
    this.drivetrain = drivetrain;
  }

  // Returns true when the command should end.

  @Override
  public void execute() {
    SmartDashboard.putNumber("balance error", this.getController().getPositionError());
    SmartDashboard.putNumber("balance set point", this.getController().getSetpoint());
    SmartDashboard.putNumber("balance  pitch", drivetrain.getPitch());
    SmartDashboard.putBoolean("balance isfinished", Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance());
    super.execute();
  }

  @Override
  public boolean isFinished() {
    //return Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.arcadeDrive(0, 0);
  }
}
