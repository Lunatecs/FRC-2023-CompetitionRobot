// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTurnCommand extends PIDCommand {

  private DrivetrainSubsystem drive;
  /** Creates a new AutoTurnCommand. */
  public AutoTurnCommand(DrivetrainSubsystem drive, double rotation) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> drive.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> rotation,
        // This uses the output
        output -> {
          //drive.arcadeDrive(0, output);

          drive.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drive);
    this.getController().setTolerance(1);
    this.drive = drive;
  }

  @Override
  public void initialize(){
    super.initialize();
    drive.resetPigeon();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
