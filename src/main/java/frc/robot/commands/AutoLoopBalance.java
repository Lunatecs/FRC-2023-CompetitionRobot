// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLoopBalance extends CommandBase {
  /** Creates a new AutoLoopBalance. */
  DrivetrainSubsystem drive;
  Double pitch;
  boolean check = true;

  public AutoLoopBalance(DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();  
    pitch = drive.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(check){
      new AutoMoveCommand(2, drive, 0.2, 0.06);
      if(pitch == 0){
        check = false;
      }
      /*drive.curvatureDrive(0.5, 0, true);
      new WaitCommand(0.3);
      drive.curvatureDrive(0, 0, true); <-- back up method ngl - hunter*/
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
