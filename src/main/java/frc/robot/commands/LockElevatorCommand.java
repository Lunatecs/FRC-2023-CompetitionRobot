// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LockElevatorCommand extends PIDCommand {
  SetPointSupplier setpoint;
  ElevatorSubsystem elevator;

  /** Creates a new LockElevatorCommand. */
  public LockElevatorCommand(SetPointSupplier setpoint, ElevatorSubsystem elevator) {
    super(
        // The controller that the command will use
        new PIDController(0.0001, 0, 0),
        // This should return the measurement
        () -> elevator.getElevatorEncoder(),
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          // Use the output here
          elevator.setElevatorSpeed(output, 0); //TODO: update position later
        });
    
    addRequirements(elevator);
    this.setpoint = setpoint;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    setpoint.setSetPoint(elevator.getElevatorEncoder());
    super.initialize();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("elevator error", this.getController().getPositionError());
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
