// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX leftMotor = new WPI_TalonSRX(IntakeConstants.LEFT_MOTOR);
  private WPI_TalonSRX rightMotor = new WPI_TalonSRX(IntakeConstants.RIGHT_MOTOR);

  private final DoubleSolenoid frontIntake = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    IntakeConstants.FORWARD_CHANNEL, 
    IntakeConstants.REVERSE_CHANNEL);

  public IntakeSubsystem() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);// does this need to be here?

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.follow(leftMotor);

    frontIntake.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // ConeIntake is the narrow position of the intake used to grip onto cones
  //public void ConeIntake(){}
  // CubeIntake is the wide position of the intake used to grip onto cubes
  //public void CubeIntake(){}

  public void intakeOpen(){
    this.frontIntake.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeClose(){
    this.frontIntake.set(DoubleSolenoid.Value.kReverse);
  }

  public void runIntake(double intakeSpeed) {
    leftMotor.set(intakeSpeed);
  }
}
