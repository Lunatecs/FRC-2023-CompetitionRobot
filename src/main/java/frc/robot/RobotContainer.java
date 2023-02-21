// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoChargingStation;
import frc.robot.commands.AutoMoveCommand;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LockElevatorCommand;
import frc.robot.commands.LooneyDriveCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
//import frc.robot.commands.ToggleLED;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final Joystick testJoystick = new Joystick(JoystickConstants.TEST_USB);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private SetPointSupplier elevatorSetpoint = new SetPointSupplier();
  boolean isCone = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
    configureAutos();
    //CameraServer.startAutomaticCapture();
  }

  public void configureAutos() {
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Move Forward", new AutoMoveCommand(145, drivetrain, 0.5, 0.25));
    autoChooser.addOption("Auto Charge Station", new AutoChargingStation(drivetrain));
    SmartDashboard.putData(autoChooser);
  }

  public void configureDefaultCommands() {
    // add keybinds for turnInPlace, fast, and reverse
    drivetrain.setDefaultCommand(new LooneyDriveCommand(drivetrain,
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS);}, 
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_X_AXIS);}, 
    () -> driverJoystick.getRawButton(JoystickConstants.RIGHT_BUMPER), 
    () -> driverJoystick.getRawButton(JoystickConstants.LEFT_BUMPER)));

    //led.setDefaultCommand(new ToggleLED(led));
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

    //Driver Controller Button Bindings
    //Intake
    new Trigger(() -> {return Math.abs(driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER)) > 0.1;}).onTrue(new RunIntakeCommand(() -> -driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER), intake))
                                                                                                            .onFalse(new RunCommand(() -> intake.runIntake(0), intake));

    new Trigger(() -> {return Math.abs(driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER)) > 0.1;}).onTrue(new RunIntakeCommand(() -> driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER), intake))
                                                                                                          .onFalse(new RunCommand(() -> intake.runIntake(0), intake));


    //Auto Balance Button
    new JoystickButton(driverJoystick, JoystickConstants.RED_BUTTON).onTrue(new AutoBalanceCommand(drivetrain, .5))
                                                                    .onFalse(new InstantCommand(() -> {}, drivetrain));

    //Operator Controller Button Bindings
    //Manual Elevator Buttons
    new POVButton(operatorJoystick, JoystickConstants.POV_UP).whileTrue(new RepeatCommand(new RunCommand(() -> elevator.setManualSpeed(-1), elevator)))
                                                                              //.onFalse(new LockElevatorCommand(new SetPointSupplier(), elevator));
                                                                              .onFalse(new RunCommand(() -> elevator.lockElevator(), elevator));
    
    new POVButton(operatorJoystick, JoystickConstants.POV_DOWN).whileTrue(new RepeatCommand(new RunCommand(() -> elevator.setManualSpeed(1), elevator)))
                                                                              .onFalse(new RunCommand(() -> elevator.lockElevator(), elevator));

    //Elevator Setpoints
    new JoystickButton(operatorJoystick, JoystickConstants.GREEN_BUTTON).onTrue(new SetElevatorPositionCommand(elevator, ElevatorConstants.BOTTOM, 0.0));
                                                                        //.onFalse(new InstantCommand(() -> {}, elevator));

    new JoystickButton(operatorJoystick, JoystickConstants.YELLOW_BUTTON).onTrue(new SetElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.0));
                                                                          //.onFalse(new InstantCommand(() -> {}, elevator));

    new JoystickButton(operatorJoystick, JoystickConstants.BLUE_BUTTON).onTrue(new SetElevatorPositionCommand(elevator, ElevatorConstants.MID_HEIGHT, 0.0));
                                                                        //.onFalse(new InstantCommand(() -> {}, elevator));
  

    //Manual Arm Buttons
    new POVButton(operatorJoystick, JoystickConstants.POV_RIGHT).whileTrue(new RepeatCommand(new RunCommand(() -> arm.setManualSpeed(-.2), arm)))
                                                                      //.onFalse(new RunCommand(() -> arm.setManualSpeed(0), arm));
                                                                      .onFalse(new RunCommand(() -> arm.setManualSpeed(0), arm));
    
    new POVButton(operatorJoystick, JoystickConstants.POV_LEFT).whileTrue(new RepeatCommand(new RunCommand(() -> arm.setManualSpeed(.2), arm)))
                                                                      //.onFalse(new RunCommand(() -> arm.setManualSpeed(0), arm));
                                                                      .onFalse(new RunCommand(() -> arm.setManualSpeed(0), arm));

    //Manual Wrist Control
    new Trigger(() -> {return Math.abs(operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)) > 0.2;}).whileTrue(new RepeatCommand(new RunCommand(() -> wrist.turnWrist(.5*operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)), wrist)))
                                                                                                            //.onFalse(new RunCommand(() -> wrist.turnWrist(0), wrist));
                                                                                                            .onFalse(new WristBrakeCommand(new SetPointSupplier(), wrist));
    
    new JoystickButton(operatorJoystick, JoystickConstants.START_BUTTON).onTrue(new WristBrakeCommand(new SetPointSupplier(), wrist))
                                                                        .onFalse(new InstantCommand(() -> {}, wrist));

    //new JoystickButton(driverJoystick, JoystickConstants.BACK_BUTTON).onTrue(new PrintCommand(led.printQueue()));

  
    //new JoystickButton(driverJoystick, JoystickConstants.BACK_BUTTON).toggleOnTrue(new RunCommand(() -> led.addColor(led.INTAKE_CONE), led));
                                                                        //.toggleOnFalse(new RunCommand(() -> led.removeColor(led.INTAKE_CONE), led));                                                             

    // some test code rn dont worry
    new JoystickButton(testJoystick, JoystickConstants.BACK_BUTTON)
                      .onTrue(new RunCommand(() -> {
                        led.removeColor(led.INTAKE_CONE);
                        led.addColor(led.INTAKE_CUBE);
                      }, led));

    new JoystickButton(testJoystick, JoystickConstants.START_BUTTON)
                      .onTrue(new RunCommand(() -> {
                        led.removeColor(led.INTAKE_CUBE);
                        led.addColor(led.INTAKE_CONE);
                      }, led));
    

    new JoystickButton(testJoystick, JoystickConstants.BLUE_BUTTON).onTrue(new PrintCommand(led.printQueue())); 

    new JoystickButton(testJoystick, JoystickConstants.YELLOW_BUTTON).onTrue(new RunCommand(() -> drivetrain.coastMode(true), drivetrain))
                                                                     .onFalse(new RunCommand(() -> drivetrain.coastMode(false), drivetrain));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    

