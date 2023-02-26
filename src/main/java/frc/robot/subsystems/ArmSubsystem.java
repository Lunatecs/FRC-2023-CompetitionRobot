// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. 
   * (The arm is what moves the intake back and forth)
  */
  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(ArmConstants.ARM_MOTOR);
  private final DigitalInput armLimitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH);
  private final CANCoder armEncoder = new CANCoder(ArmConstants.ARM_ENCODER);
  private PIDController pidController = null;

  public ArmSubsystem() {
    armMotor.configFactoryDefault();
    armMotor.setNeutralMode(NeutralMode.Brake);
    armEncoder.configFactoryDefault();
    CANCoderConfiguration config = new CANCoderConfiguration();
    armEncoder.configAllSettings(config);
    armEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
    pidController = new PIDController(0.01, 0.0, 0.0);
    pidController.setTolerance(40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmEncoder());
    SmartDashboard.putBoolean("arm limit switch", getArmLimitSwitch());
    SmartDashboard.putBoolean("Arm: Red = Bottom", armLimitSwitch.get());
    SmartDashboard.putNumber("arm pid output", pidController.calculate(getArmEncoder()));

    if(!armLimitSwitch.get()) {
      this.resetEncoders();
    }

   //if (!getArmLimitSwitch()) {
   // this.resetEncoders();
   //}
  }

/* 
  public void setSpeed(double speed) {
    if (speed < 0 &&)
    
  }*/

  public void setSpeed(double speed) {   //Asumming that motor works the same as the elevator
      armMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

  public boolean getArmLimitSwitch() {
    return armLimitSwitch.get();
  }

  public void lockArm() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }
 
  public void resetEncoders() {
    armEncoder.setPosition(0);
  }
}
