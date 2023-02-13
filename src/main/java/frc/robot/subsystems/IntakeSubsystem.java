// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
 
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.RIGHT_MOTOR);

  /*private final DoubleSolenoid frontIntake = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    IntakeConstants.FORWARD_CHANNEL, 
    IntakeConstants.REVERSE_CHANNEL);*/

  public IntakeSubsystem() {
    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  


  // The intakeOpen() and intakeClose() methods are designed for Seth's intake
  /*public void intakeOpen(){
    this.frontIntake.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeClose(){
    this.frontIntake.set(DoubleSolenoid.Value.kReverse);
  }*/

  public void runIntake(double intakeSpeed) {
    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
  }
}
