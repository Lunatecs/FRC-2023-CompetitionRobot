// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class DrivetrainConstants{
    public final static int LEFT_FRONT = 2;    
    public final static int LEFT_BACK = 4;
    public final static int RIGHT_FRONT = 3;
    public final static int RIGHT_BACK = 1;

    public final static int PIGEON = 6;



    public final static double WHEEL_DIAMETER = 0.1524;
    public final static double TICKS = 2048.0;
    public final static double GEAR_REDUCTION = 10;
    public final static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final static double METERS_PER_TICK = WHEEL_CIRCUMFERENCE / (TICKS * GEAR_REDUCTION);

   /* 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  */
  }

  public final static class IntakeConstants{
    public final static int LEFT_MOTOR = 1; // TODO: Find Intake motor and solenoid CAN values
    public final static int RIGHT_MOTOR = 1;

    public final static int FORWARD_CHANNEL = 1;
    public final static int REVERSE_CHANNEL = 1;
  }

  public final static class ElevatorConstants{
    public final static int ELEVATOR_MOTOR = 1;
  }

  public final static class ArmConstants {
    public final static int ARM_MOTOR = 0; // TODO: find the CAN id for arm motor and limit switch (once its attached)
    public final static int LIMIT_SWITCH = 0;
  }

  public final static class JoystickConstants{
    
    public final static int DRIVER_USB = 0;
    public final static int OPERATOR_USB = 1;
    
    
    public final static int LEFT_Y_AXIS = 1;
    public final static int RIGHT_X_AXIS = 4;


    public final static int GREEN_BUTTON = 1;
    public final static int RED_BUTTON = 2;
    public final static int YELLOW_BUTTON = 4;
    public final static int BLUE_BUTTON = 3;

    public final static int LEFT_TRIGGER = 2;
    public final static int RIGHT_TRIGGER = 3;
    public final static int LEFT_BUMPER = 5;
    public final static int RIGHT_BUMPER = 6;

    public final static int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
  }
}
