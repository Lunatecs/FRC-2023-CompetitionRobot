// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

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

  private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DrivetrainConstants.PIGEON);

  private  double zeroAngle;

  public final double zeroYaw;

  private final DifferentialDriveOdometry odometry;

  private DifferentialDrive drive;

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

    pigeon.reset();
    pigeon.calibrate();
    zeroAngle = pigeon.getPitch();
    zeroYaw = pigeon.getYaw();
    odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), getLeftDistance(), getRightDistance()); //Copied from wpi docs so its hit or miss
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
    SmartDashboard.putNumber("Yaw", getYaw());

    

  }
  public double getLeftDistance() {
    return leftFront.getSelectedSensorPosition();
  } 

  public double getRightDistance() {
    return rightFront.getSelectedSensorPosition();
  } 
 
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    getLeftSpeed(), 
    getRightSpeed());
  }

  public double getLeftSpeed() {
    return leftFront.getSelectedSensorVelocity();
  }

  public double getRightSpeed() {
    return rightFront.getSelectedSensorVelocity();
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

  public double getPitch() {
    double [] yawPitchRoll = new double[3];
    pigeon.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[1];
  }

  public double getYaw() {
    double [] yawPitchRoll = new double[3];
    pigeon.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[0];
  }

  public double getZeroAngle() {
    return zeroAngle;
  }

  public double getZeroYaw(){
    return zeroYaw;
  }

  public void setYawToZero() {
    pigeon.setYaw(0);
  }

  public void resetPigeon(){
    pigeon.reset();
    
    //zeroAngle = pigeon.getPitch();
  }
  

}