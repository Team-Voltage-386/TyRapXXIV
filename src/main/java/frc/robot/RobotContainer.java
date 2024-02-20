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
import frc.robot.Subsystems.CameraSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.PickupMotorsSubsystem;
import frc.robot.Subsystems.PickupOrchestrator;
import frc.robot.Subsystems.PneumaticsSubsystem;
import frc.robot.Commands.Drive;
import frc.robot.Commands.StopDrive;
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
  public final Drivetrain m_swerve = new Drivetrain(m_gyro, m_cameraSubsystem);
  public final PickupMotorsSubsystem m_pickupMotors = new PickupMotorsSubsystem();
  public final PneumaticsSubsystem m_pneumatics = new PneumaticsSubsystem();

  public final PickupOrchestrator m_pickup = new PickupOrchestrator(m_pneumatics, m_pickupMotors);

  Command driveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));

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

    // drive cont bindings
    Controller.kDriveController.rightBumper().onTrue((new resetOdo(m_swerve)));
    Controller.kDriveController.rightTrigger(0.25).toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

    // Temporary
    Controller.kManipulatorController.a().and(m_pickup.noPieceTrigger).onTrue(m_pickup.runIntakeCommand());
    Controller.kManipulatorController.b().onTrue(m_pickup.disableIntakeCommand());
    // Controller.kManipulatorController.x().onTrue(m_pickup.loadPieceCommand());
    Controller.kManipulatorController.y().onTrue(m_pickup.lowerLoaderCommand());

    Controller.kManipulatorController.rightBumper()
        .onTrue(new SequentialCommandGroup(Commands.runOnce(() -> System.out.println(Timer.getFPGATimestamp())),
            new TimerWaitCommand(1), Commands.runOnce(() -> System.out.println(Timer.getFPGATimestamp()))));
  }

  public Drivetrain getDrivetrain() {
    return m_swerve;
  }

  public Command getTeleOpCommand() {
    return new ParallelCommandGroup(driveCommand);
  }

  public void print() {
    this.m_swerve.print();
  }
}
