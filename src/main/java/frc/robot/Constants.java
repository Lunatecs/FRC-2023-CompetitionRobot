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

    public final static int PIGEON = 24; 
    
    public final static double WHEEL_DIAMETER = 6.0;
    public final static double TICKS = 2048.0;// 1 rotation (6 inches)
    public final static double GEAR_RATIO = 10;
    public final static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final static double TICKS_PER_INCH = (TICKS*GEAR_RATIO)/WHEEL_CIRCUMFERENCE; 

   /* 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  */
  }

  public final static class IntakeConstants{
    public final static int INTAKE_MOTOR = 9;
  }

  public final static class ElevatorConstants{
    public final static int ELEVATOR_MOTOR = 5;
    public static final int LIMIT_SWITCH = 9;
    public static final double MAX_HEIGHT = -86000;//-95000;//-95000;//-98000;//-101000; //-182000 (old)
    public static final double STATION_HEIGHT = -101000;
    public static final double MID_HEIGHT = -53000;//-57000;//-60000;//-63000; //-135972 (old)
    public static final double BOTTOM = 0;
  }

  public final static class ArmConstants {
    public final static int ARM_MOTOR = 8; 
    public final static int LIMIT_SWITCH = 8;
    public final static double MAX_EXTENSION = 740.0;
    public final static double TOP_EXTENSION = 494;
    public final static String Joe = "Joe mother";
  }

  public final static class JoystickConstants{
    
    public final static int DRIVER_USB = 0;
    public final static int OPERATOR_USB = 1;
    public final static int TEST_USB = 2;
    
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
    public final static int START_BUTTON = 8;

    public final static int POV_UP = 0;
    public final static int POV_RIGHT = 90;
    public final static int POV_DOWN = 180;
    public final static int POV_LEFT = 270;
  }

  public final static class WristConstants{
    public final static int WRIST_MOTOR = 6;
    public final static double WRIST_HOME = 5000;
    public final static double CONE_SETPOINT = 24000;
    public final static double GROUND_INTAKE_CONE = 40000;
    public final static double GROUND_INTAKE_CUBE = 47000; 
    //These will be made for cones, but it is likely that cubes will require a seperate constant.
    //public final static double STATION_INTAKE = ;
  }

  public final static class LEDConstants {
    public final static int LED_FRONT = 0;
    public final static int LED_BACK = 1;

    public final static double STROBE_GOLD = -0.07;
    public final static double STROBE_BLUE = -0.09;
    public final static double FIRE_MED =  -0.59;
    public final static double SOLID_GREEN = 0.71;
    public final static double CONFETTI = -0.87;
    public final static double SOLID_VIOLET = 0.91;
  }

}
