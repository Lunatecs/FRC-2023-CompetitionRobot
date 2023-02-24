// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  private WPI_TalonFX elevatorMotor = new WPI_TalonFX(ElevatorConstants.ELEVATOR_MOTOR);
  private final DigitalInput elevatorLimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH);
  private PIDController pidController = null;

  public ElevatorSubsystem() {
    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    pidController = new PIDController(0.0001, 0.0, 0.0);
    pidController.setTolerance(500);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator encoder", getElevatorEncoder());
    SmartDashboard.putBoolean("Elevator: Red = Bottom", elevatorLimitSwitch.get());
    SmartDashboard.putNumber("elevator error", this.pidController.getPositionError());
    SmartDashboard.putNumber("elevator setpoint", this.pidController.getSetpoint());
    SmartDashboard.putNumber("pidController output", this.pidController.calculate(getElevatorEncoder()));
    if(!elevatorLimitSwitch.get()) {
      this.resetEncoders();
    }
  }

  public void setManualSpeed(double speed) {
    if(speed > 0) { //If elevator is going down...
      pidController.setSetpoint(0);
      elevatorMotor.set(ControlMode.PercentOutput, pidController.calculate(getElevatorEncoder()));
    } else if(speed < 0 && !tripLimit()) { //If elevator is going up...
      pidController.setSetpoint(ElevatorConstants.MAX_HEIGHT);
      elevatorMotor.set(ControlMode.PercentOutput, pidController.calculate(getElevatorEncoder()));
    } else {
      elevatorMotor.set(ControlMode.PercentOutput, 0);
    }
        /*
     * if elevator up:
     *  max height pidloop
     *  when reaches max, stops if continues going up
     * if elevator down:
     *  min height pidloop until hits bottom
     *  when hits bottom:
     *    reset encoder
     *    set speed to 0 if continue going down
     */
  }

  public void setManualSpeed(double speed, double setpoint) {
    if(speed > 0) { //If elevator is going down...
      pidController.setSetpoint(setpoint);
      elevatorMotor.set(ControlMode.PercentOutput, pidController.calculate(getElevatorEncoder()));
    } else if(speed < 0 && !tripLimit()) { //If elevator is going up...
      pidController.setSetpoint(ElevatorConstants.MAX_HEIGHT);
      elevatorMotor.set(ControlMode.PercentOutput, pidController.calculate(getElevatorEncoder()));
    } else {
      elevatorMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setSpeed(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void lockElevator() {
    elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getElevatorEncoder() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public boolean tripLimit() {
    if(elevatorMotor.getStatorCurrent() > 100) {
      return true;
    }
    return false;
  }

  public void resetEncoders() {
    elevatorMotor.setSelectedSensorPosition(0.0);
  }

  public void coastMode(boolean coast) {
    if(coast) {
      elevatorMotor.setNeutralMode(NeutralMode.Coast);
      return;
    }
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

}
