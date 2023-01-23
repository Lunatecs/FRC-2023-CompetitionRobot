// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  private WPI_TalonFX elevatorMotor = new WPI_TalonFX(ElevatorConstants.ELEVATOR_MOTOR);


  public ElevatorSubsystem() {
    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public double getElevatorEncoder() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public void resetEncoders() {
    elevatorMotor.setSelectedSensorPosition(0.0);
  }
}
