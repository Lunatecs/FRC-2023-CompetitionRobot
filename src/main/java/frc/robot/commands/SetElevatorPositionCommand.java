// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorPositionCommand extends PIDCommand {
  ElevatorSubsystem elevator;
  public SetElevatorPositionCommand(ElevatorSubsystem elevator, double setpoint, double i) {
    super(
        // The controller that the command will use
        new PIDController(0.0001, i, 0),
        // This should return the measurement
        () -> elevator.getElevatorEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          elevator.setSpeed(output);
        });
        addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    this.getController().setTolerance(500);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Elevator Error", this.getController().getPositionError());
    SmartDashboard.putNumber("Elevator Tolerance", this.getController().getPositionTolerance());
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
