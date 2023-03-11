// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Due to the nature of the limelight's mounting, we will be swapping the X and Y values (Limelight rotated 90 degrees)
  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }

  public double getTY(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
  }

  public void setPipeline(int pipeline){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  public void setLights(boolean isOn) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }
}
