// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ArcadeDriveCommand;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LooneyDriveCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.WristBrakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.SetPointSupplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final WristSubsystem wrist = new WristSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final LEDSubsystem led = LEDSubsystem.getInstance();



  private final Joystick driverJoystick = new Joystick(Constants.JoystickConstants.DRIVER_USB);
  private final Joystick operatorJoystick = new Joystick(Constants.JoystickConstants.OPERATOR_USB);

  boolean isCone = true;
  boolean isCube = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
    //CameraServer.startAutomaticCapture();
  }


  public void configureDefaultCommands() {
    // add keybinds for turnInPlace, fast, and reverse
    drivetrain.setDefaultCommand(new LooneyDriveCommand(drivetrain,
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS);}, 
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_X_AXIS);}, () -> {return false;}, () -> {return false;}, () -> {return false;}));
  
    led.setDefaultCommand(new PrintCommand(led.printQueue()));

    //wrist.setDefaultCommand(new WristBrakeCommand(new SetPointSupplier(), wrist));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    new JoystickButton(operatorJoystick, JoystickConstants.YELLOW_BUTTON).whileTrue(new RepeatCommand(new RunCommand(() -> elevator.setElevatorSpeed(1), elevator)))
                                                                              .onFalse(new RunCommand(() -> elevator.setElevatorSpeed(0), elevator));
    
    new JoystickButton(operatorJoystick, JoystickConstants.GREEN_BUTTON).whileTrue(new RepeatCommand(new RunCommand(() -> elevator.setElevatorSpeed(-1), elevator)))
                                                                              .onFalse(new RunCommand(() -> elevator.setElevatorSpeed(0), elevator));
  
    new Trigger(() -> {return Math.abs(driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER)) > 0.1;}).onTrue(new RunIntakeCommand(() -> -driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER), intake))
                                                                                                         .onFalse(new RunCommand(() -> intake.runIntake(0), intake));

    new Trigger(() -> {return Math.abs(driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER)) > 0.1;}).onTrue(new RunIntakeCommand(() -> driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER), intake))
                                                                                                          .onFalse(new RunCommand(() -> intake.runIntake(0), intake));
    
    new JoystickButton(operatorJoystick, JoystickConstants.RED_BUTTON).whileTrue(new RepeatCommand(new RunCommand(() -> arm.setArmSpeed(-.8), arm)))
                                                                      .onFalse(new RepeatCommand(new RunCommand(() -> arm.setArmSpeed(0), arm)));
    
    new JoystickButton(operatorJoystick, JoystickConstants.BLUE_BUTTON).whileTrue(new RepeatCommand(new RunCommand(() -> arm.setArmSpeed(.8), arm)))
                                                                      .onFalse(new RepeatCommand(new RunCommand(() -> arm.setArmSpeed(0), arm)));

    //new JoystickButton(operatorJoystick, JoystickConstants.LEFT_Y_AXIS).whileTrue(new RepeatCommand(new RunCommand(() -> wrist.turnWrist(.5*operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)), wrist)))
    //
    new Trigger(() -> {return Math.abs(operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)) > 0.2;}).whileTrue(new RepeatCommand(new RunCommand(() -> wrist.turnWrist(.5*operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)), wrist)))
                                                                                                            //.onFalse(new RunCommand(() -> wrist.turnWrist(0), wrist));
                                                                                                            .onFalse(new WristBrakeCommand(new SetPointSupplier(), wrist));
    
    new JoystickButton(operatorJoystick, JoystickConstants.START_BUTTON).onTrue(new WristBrakeCommand(new SetPointSupplier(), wrist))
                                                                        .onFalse(new InstantCommand(() -> {}, wrist));

    new JoystickButton(driverJoystick, JoystickConstants.START_BUTTON).toggleOnTrue(new RunCommand(() -> led.addColor(led.INTAKE_CUBE), led))
                                                                      .toggleOnFalse(new RunCommand(() -> led.removeColor(led.INTAKE_CUBE), led));
                                                                                                                                              

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
