// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private WPI_TalonFX leftFront = new WPI_TalonFX(DrivetrainConstants.LEFT_FRONT);
  private WPI_TalonFX leftBack = new WPI_TalonFX(DrivetrainConstants.LEFT_BACK);
  private WPI_TalonFX rightFront = new WPI_TalonFX(DrivetrainConstants.RIGHT_FRONT);
  private WPI_TalonFX rightBack = new WPI_TalonFX(DrivetrainConstants.RIGHT_BACK);

  //private Gyro gyro =  new WPI_PigeonIMU(new TalonSRX(Constants.DrivetrainConstants.PIGEON));
  private DifferentialDrive drive;
  //This maybe be screwed up, cannot figure out distance parameter thingys
  //private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

  public DrivetrainSubsystem() {
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    rightFront.setInverted(false);
    rightBack.setInverted(false);
    leftFront.setInverted(true);
    leftBack.setInverted(true);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(leftFront, rightFront);

  }
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public void curvatureDrive(double speed, double rotation, boolean turnInPlace) {
    drive.curvatureDrive(speed*-1.0, rotation*-1.0, turnInPlace);
  }

  public void voltageTankDrive(double leftVolts, double rightVolts) {
    rightFront.setVoltage(rightVolts);                              
    leftFront.setVoltage(leftVolts);                                 
    drive.feed();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
 //   SmartDashboard.putNumber("Gyro", this.getAngle());

  }
  public double getLeftDistance() {
    return (leftFront.getSelectedSensorPosition() * DrivetrainConstants.METERS_PER_TICK);
  } 

  public double getRightDistance() {
    return (rightFront.getSelectedSensorPosition() * DrivetrainConstants.METERS_PER_TICK);
  } 
/* 
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
*/
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    getLeftSpeed(), 
    getRightSpeed());
  }

  public double getLeftSpeed() {
    return leftFront.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_TICK * 10;
  }

  public double getRightSpeed() {
    return rightFront.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_TICK * 10;
  } 
/* 
  public double getAngle() {
    return this.gyro.getAngle();
  }
*/
  public void resetEncoders() {
    rightFront.setSelectedSensorPosition(0.0);
    leftFront.setSelectedSensorPosition(0.0);
  }

  public void coastMode(boolean coast) {
    if(coast) {
      leftFront.setNeutralMode(NeutralMode.Coast);
      leftBack.setNeutralMode(NeutralMode.Coast);
      rightFront.setNeutralMode(NeutralMode.Coast);
      rightBack.setNeutralMode(NeutralMode.Coast);
      return;
    }
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
  }

}