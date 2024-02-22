// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
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
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
  private static final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
  private final Drivetrain m_swerve = new Drivetrain(m_gyro, m_cameraSubsystem);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final Aimlock m_aim = new Aimlock(m_swerve, m_shooter);
  public final PickupMotorsSubsystem m_pickupMotors = new PickupMotorsSubsystem();
  public final PneumaticsSubsystem m_pneumatics = new PneumaticsSubsystem();

  public final PickupOrchestrator m_pickup = new PickupOrchestrator(m_pneumatics, m_pickupMotors);

  Command driveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));

    m_swerve.setAim(m_aim);
    m_shooter.setAim(m_aim);
    m_shooter.setDefaultCommand(new aimShooterCommand(m_shooter));

    // Xbox controllers return negative values when we push forward.
    // Xbox controllers return negative values when we push forward.
    driveCommand = new Drive(m_swerve);
    m_swerve.setDefaultCommand(driveCommand);

    // Register named commands
    NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve)); // dont need anymore (?)
    NamedCommands.registerCommand("Lock Target in Auto",
        Commands.runOnce(() -> m_swerve.setLockTargetInAuto(true), m_swerve));
    NamedCommands.registerCommand("Dont Lock Target in Auto",
        Commands.runOnce(() -> m_swerve.setLockTargetInAuto(false), m_swerve));

    // Configure the trigger bindings
    configureBindings();
    // Configure the button bindings
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
        .onTrue(Commands.runOnce(() -> m_shooter.shootToggle(), m_shooter));
    // .onFalse(new ParallelCommandGroup(Commands.runOnce(() ->
    // m_shooter.shootToggle(), m_shooter),
    // m_pickup.lowerLoaderCommand()));
    new Trigger(m_shooter::hasShotNote)
        .onTrue(new SequentialCommandGroup(new TimerWaitCommand(0.25),
            new ParallelCommandGroup(Commands.runOnce(() -> m_shooter.shootToggle(), m_shooter),
                m_pickup.lowerLoaderCommand())));
    Controller.kManipulatorController.povLeft()
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.AMP)));
    Controller.kManipulatorController.povRight()
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.SPEAKER)));
    Controller.kManipulatorController.povDown()
        .whileTrue(Commands.run(() -> m_pickupMotors.runMotorsReverse(), m_pickupMotors))
        .onFalse(m_pickupMotors.stopMotorsCommand());
    Controller.kManipulatorController.a().and(m_pickup.noPieceTrigger).onTrue(m_pickup.runIntakeCommand());
    Controller.kManipulatorController.b().onTrue(m_pickup.disableIntakeCommand());
    // Controller.kManipulatorController.x().onTrue(m_pickup.loadPieceCommand());
    Controller.kManipulatorController.y().onTrue(m_pickup.lowerLoaderCommand());
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
    // Controller.kDriveController.rightTrigger(0.25).onTrue(m_pickup.runIntakeCommand())
    // .onFalse(m_pickup.disableIntakeCommand());
    Controller.kDriveController.rightTrigger(0.25).toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());
    // Temporary

    // Controller.kManipulatorController.x()

    new Trigger(() -> m_shooter.getBottomLimit())
        .onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(0)));
    new Trigger(() -> m_shooter.getTopLimit()).onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(
        20)));
  }

  public Drivetrain getDrivetrain() {
    return m_swerve;
  }

  public ShooterSubsystem getShooter() {
    return m_shooter;
  }

  public Command getTeleOpCommand() {
    return new ParallelCommandGroup(driveCommand, Commands.runOnce(m_shooter::setAimToBreakMode, m_shooter));
  }

  public void print() {
    this.m_swerve.print();
  }
}
