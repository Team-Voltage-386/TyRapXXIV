// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controller;
import frc.robot.Constants.ID;
import frc.robot.Constants.PipeLineID;
import frc.robot.Subsystems.CameraSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Utils.Aimlock;
import frc.robot.Subsystems.PickupMotorsSubsystem;
import frc.robot.Subsystems.PickupOrchestrator;
import frc.robot.Subsystems.PneumaticsSubsystem;
import frc.robot.Commands.Drive;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.aimShooterCommand;
import frc.robot.Commands.lockTarget;
import frc.robot.Commands.TimerWaitCommand;
import frc.robot.Commands.resetOdo;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
  private final CameraSubsystem m_cameraSubsystem;
  private final Drivetrain m_swerve;
  private final ShooterSubsystem m_shooter;
  private final Aimlock m_aim;
  private final PickupMotorsSubsystem m_pickupMotors;
  private final PneumaticsSubsystem m_pneumatics;

  private final PickupOrchestrator m_pickup;

  Command driveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
    this.m_cameraSubsystem = new CameraSubsystem();
    this.m_swerve = new Drivetrain(m_gyro, m_cameraSubsystem);
    this.m_shooter = new ShooterSubsystem();
    this.m_aim = new Aimlock(m_swerve, m_shooter);
    this.m_pickupMotors = new PickupMotorsSubsystem();
    this.m_pneumatics = new PneumaticsSubsystem();
    this.m_pickup = new PickupOrchestrator(m_pneumatics, m_pickupMotors);

    m_swerve.setAim(m_aim);
    m_shooter.setAim(m_aim);
    m_shooter.setDefaultCommand(new aimShooterCommand(m_shooter));

    // Xbox controllers return negative values when we push forward.
    driveCommand = new Drive(m_swerve);
    m_swerve.setDefaultCommand(driveCommand);

    // Register named commands
    NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve)); // dont need anymore (?)
    NamedCommands.registerCommand("Lock Target in Auto",
        Commands.runOnce(() -> m_swerve.setLockTargetInAuto(true), m_swerve));
    NamedCommands.registerCommand("Dont Lock Target in Auto",
        Commands.runOnce(() -> m_swerve.setLockTargetInAuto(false), m_swerve));
    NamedCommands.registerCommand("Shoot", Commands.runOnce(m_shooter::shoot));
    NamedCommands.registerCommand("Dont Shoot", Commands.runOnce(m_shooter::noShoot));
    NamedCommands.registerCommand("Intake Down", m_pickup.runIntakeCommand());
    NamedCommands.registerCommand("Intake Up", m_pickup.disableIntakeCommand());

    // Configure the trigger bindings
    configureBindings();
    // Configure the button bindings

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()'
    // Create choices for autonomous functions in the Smart Dashboard
    configPathPlannerStuff();
    configureBindings();
    autoChooser.setDefaultOption("Autonomous Command", auto1);
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Controller.kManipulatorController.rightBumper()
        .onTrue(Commands.runOnce(() -> m_shooter.shoot())).onFalse(Commands.runOnce(() -> m_shooter.noShoot()));
    // .onFalse(new ParallelCommandGroup(Commands.runOnce(() ->
    // m_shooter.shootToggle(), m_shooter),
    // m_pickup.lowerLoaderCommand()));
    new Trigger(() -> m_shooter.hasShotNote())
        .onTrue(new SequentialCommandGroup(new TimerWaitCommand(0.25),
            new ParallelCommandGroup(Commands.runOnce(() -> m_shooter.noShoot(),
                m_shooter)),
            m_pickup.lowerLoaderCommand()));

    Controller.kManipulatorController.povLeft()
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.AMP)));
    Controller.kManipulatorController.povRight()
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.SPEAKER)));
    Controller.kManipulatorController.povDown()
        .whileTrue(Commands.run(() -> m_pickupMotors.runMotorsReverse(),
            m_pickupMotors))
        .onFalse(m_pickupMotors.stopMotorsCommand());
    Controller.kManipulatorController.a().and(m_pickup.noPieceTrigger).onTrue(m_pickup.runIntakeCommand());
    Controller.kManipulatorController.b().onTrue(m_pickup.disableIntakeCommand());
    // Controller.kManipulatorController.x().onTrue(m_pickup.loadPieceCommand());
    Controller.kManipulatorController.y().onTrue(m_pickup.lowerLoaderCommand());
    // Controller.kManipulatorController.x().whileTrue(pathfindAmp);

    // Controller.kManipulatorController.leftBumper().and(() ->
    // !m_shooter.getBottomLimit())
    // .whileTrue(Commands.runOnce(() -> m_shooter.driveShooterManually(-2.5)))
    // .onFalse(Commands.runOnce(() -> m_shooter.stopDrivingShooter()));
    // Controller.kManipulatorController.rightBumper().and(() ->
    // !m_shooter.getTopLimit())
    // .whileTrue(Commands.runOnce(() -> m_shooter.driveShooterManually(2.5)))
    // .onFalse(Commands.runOnce(() -> m_shooter.stopDrivingShooter()));
    // Controller.kDriveController.b().onTrue(m_pickup.disableIntakeCommand());

    // Controller.kDriveController.x().onTrue(Commands.runOnce(() ->
    // Aimlock.setPipeline(PipeLineID.kSpeakerID)));

    // drive cont bindings
    Controller.kDriveController.rightBumper().onTrue((new resetOdo(m_swerve)));

    Controller.kDriveController.leftBumper().onTrue(new lockTarget(m_swerve));
    // drive cont bindings
    Controller.kDriveController.a().onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(0), m_shooter));

    Controller.kDriveController.rightTrigger(0.25).toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

    // Temporary
    // Controller.kDriveController.rightTrigger(0.25).onTrue(m_pickup.runIntakeCommand())
    // .onFalse(m_pickup.disableIntakeCommand());

    // Controller.kManipulatorController.x()

    new Trigger(() -> m_shooter.getBottomLimit())
        .onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(0)));
    new Trigger(() -> m_shooter.getTopLimit()).onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(
        20)));
  }

  Command auto1;
  Command pathfindAmp;

  private void configPathPlannerStuff() {
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // SmartDashboard.putData("Example Auto", AutoBuilder.buildAuto("Example
    // Auto"));
    // Add a button to run a simple example path
    auto1 = AutoBuilder.buildAuto("tyrap test");
    autoChooser.addOption("auto1", auto1);
    // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile("Score Amp");
    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        1, 3.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindAmp = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  public Drivetrain getDrivetrain() {
    return m_swerve;
  }

  public ShooterSubsystem getShooter() {
    return m_shooter;
  }

  public Command getTeleOpCommand() {
    return new ParallelCommandGroup(driveCommand,
        Commands.runOnce(m_shooter::setAimToBreakMode, m_shooter));
  }

  public void print() {
    this.m_swerve.print();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ParallelCommandGroup(auto1);
  }
}
