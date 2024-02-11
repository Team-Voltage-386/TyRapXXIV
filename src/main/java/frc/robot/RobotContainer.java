// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.databind.module.SimpleKeyDeserializers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controller;
import frc.robot.Constants.ID;
import frc.robot.Subsystems.CameraSubsystem.CameraSourceOption;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Aimlock.DoState;
import frc.robot.Subsystems.CameraSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.PickupSubsystem;
import frc.robot.Subsystems.PneumaticSubsystem;
import frc.robot.Subsystems.RumbleSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Commands.Drive;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.lockTarget;
import frc.robot.Commands.resetOdo;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController m_manipController = new CommandXboxController(Controller.kManipController);
  private final CommandXboxController m_driveController = new CommandXboxController(Controller.kDriveController);
  private static final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
  public final Drivetrain m_swerve = new Drivetrain(m_gyro, m_cameraSubsystem);
  // public final PneumaticSubsystem m_pneumatics = new PneumaticSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final Aimlock m_aim = new Aimlock(m_swerve, m_shooter);
  private final RumbleSubsystem m_rumble = new RumbleSubsystem(m_driveController);
  // private final PickupSubsystem m_pickup = new PickupSubsystem();
  Command driveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //pass le aimbot. once we convert all this to command based we will pass the aim to the command instead of the subsystem. make it work for now, then we will make it right!
    m_swerve.setAim(m_aim);
    m_shooter.setAim(m_aim);
    // Xbox controllers return negative values when we push forward.   
    driveCommand = new Drive(m_swerve);
    m_swerve.setDefaultCommand(driveCommand);
    
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()'
    // Create choices for autonomous functions in the Smart Dashboard
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Register named commands
    NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve)); //dont need anymore (?)
    NamedCommands.registerCommand("Lock Target in Auto", Commands.runOnce(()-> m_swerve.setLockTargetInAuto(true), m_swerve));
    NamedCommands.registerCommand("Dont Lock Target in Auto", Commands.runOnce(()-> m_swerve.setLockTargetInAuto(false), m_swerve));

    // Configure the trigger bindings
    configPathPlannerStuff();
    configureBindings();
    autoChooser.setDefaultOption("Autonomous Command", path1);
    // Configure the button bindings
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


    //drive cont bindings
    m_driveController.leftBumper().onTrue(new lockTarget(m_swerve));
    m_driveController.rightBumper().onTrue((new resetOdo(m_swerve)));
    m_driveController.x().whileTrue(pathfindAmp);
    /*
         * Y = forward camera
         * A = back camera
         */
        m_driveController.y().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.USB_CAMERA)
                .alongWith(this.m_swerve.setDirectionOptionCommand(Drivetrain.DirectionOption.BACKWARD)));
        m_driveController.b().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.FISHEYE));
        m_driveController.a().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.LIMELIGHT)
                .alongWith(this.m_swerve.setDirectionOptionCommand(Drivetrain.DirectionOption.FORWARD)));
        m_driveController.rightTrigger(0.25).toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

    //manip cont bindings
    m_manipController.x().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.NOTE)));
    m_manipController.a().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.SPEAKER)));
    m_manipController.b().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.AMP)));
    m_manipController.y().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.SOURCE)));
    new Trigger(()-> m_cameraSubsystem.isLLOdoGood(5.0)).whileTrue(m_rumble.setRumbleCommand(RumbleType.kBothRumble, 0.5, 0.25));
  }

  Command path1;
  Command pathfindAmp;
  private void configPathPlannerStuff() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    //SmartDashboard.putData("Example Auto", AutoBuilder.buildAuto("Example Auto"));
    // Add a button to run a simple example path
    path1 = AutoBuilder.buildAuto("first real auto");
    autoChooser.addOption("path", path1);
    // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile("Score Amp");
    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
      1, 3.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindAmp = AutoBuilder.pathfindThenFollowPath(
      path,
      constraints,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  // public Command toggleIntake() {
  //   return Commands.runOnce(()-> m_pneumatics.toggleIntake(), m_pneumatics);
  // }

  // public Command toggleLift() {
  //   return Commands.runOnce(()-> m_pneumatics.toggleLift(), m_pneumatics);
  // }

  public Drivetrain getDrivetrain() {
    return m_swerve;
  }

  public ShooterSubsystem getShooter() {
    return m_shooter; //
  }


  public Command resetShooterPos() {
    return Commands.runOnce(()->m_shooter.setAimPos(32), m_shooter);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return path1;
  }

  public Command getTeleOpCommand() {
    return driveCommand;
  }

  /**
   * Pathfind to the Amp
   */
  public Command pathfindAmp() {
    return pathfindAmp;
  }
}
