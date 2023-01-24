// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. 
   * (The arm is what moves the intake back and forth)
  */
  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(ArmConstants.ARM_MOTOR);
  private final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH);

  public ArmSubsystem() {
    armMotor.configFactoryDefault();
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmSpeed(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getArmEncoder() {
    return armMotor.getSelectedSensorPosition();
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }
 
}
