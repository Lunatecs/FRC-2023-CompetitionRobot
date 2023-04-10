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
public class AutoTurnWithCurrentGyro extends PIDCommand {
  /** Creates a new AutoTurnWithCurrentGyro. */

  private DrivetrainSubsystem drive;

  public AutoTurnWithCurrentGyro(DrivetrainSubsystem drive, double rotation) {
    super(
        // The controller that the command will use
        new PIDController(0.05, 0.0015, 0),
        // This should return the measurement
        () -> drive.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> rotation,
        // This uses the output
        output -> {
          // Use the output here
          if(Math.abs(output) > .5){
            output = (output/Math.abs(output)) * .5;
          }
          drive.curvatureDrive(0, output, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drive);
    this.getController().setTolerance(1);
    this.drive = drive;
  }

  @Override

  public void execute(){
    super.execute();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean fin = Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance();

    return fin;
  }
}
