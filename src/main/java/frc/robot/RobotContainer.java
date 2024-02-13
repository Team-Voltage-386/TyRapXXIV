// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
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
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Commands.Drive;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.resetOdo;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //private final CommandXboxController m_manipController = new CommandXboxController(Controller.kManipController);
  private final CommandXboxController m_driveController = new CommandXboxController(Controller.kDriveController);
  private static final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
  public final Drivetrain m_swerve = new Drivetrain(m_gyro);
  // public final PneumaticSubsystem m_pneumatics = new PneumaticSubsystem();
  // private final PickupSubsystem m_pickup = new PickupSubsystem();
  Command driveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
    
    //pass le aimbot. once we convert all this to command based we will pass the aim to the command instead of the subsystem. make it work for now, then we will make it right!
    //m_swerve.setAim(m_aim);
    //m_shooter.setAim(m_aim);
    // Xbox controllers return negative values when we push forward.   
    driveCommand = new Drive(m_swerve);
    m_swerve.setDefaultCommand(driveCommand);
    
    // Register named commands
    NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve)); //dont need anymore (?)
    NamedCommands.registerCommand("Lock Target in Auto", Commands.runOnce(()-> m_swerve.setLockTargetInAuto(true), m_swerve));
    NamedCommands.registerCommand("Dont Lock Target in Auto", Commands.runOnce(()-> m_swerve.setLockTargetInAuto(false), m_swerve));

    // Configure the trigger bindings
    configureBindings();
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
    //m_driveController.leftBumper().onTrue(new lockTarget(m_swerve));
    m_driveController.rightBumper().onTrue((new resetOdo(m_swerve)));
    /*
         * Y = forward camera
         * A = back camera
         */
        // m_driveController.y().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.USB_CAMERA)
        //         .alongWith(this.m_swerve.setDirectionOptionCommand(Drivetrain.DirectionOption.BACKWARD)));
        // m_driveController.b().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.FISHEYE));
        // m_driveController.a().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.LIMELIGHT)
        //         .alongWith(this.m_swerve.setDirectionOptionCommand(Drivetrain.DirectionOption.FORWARD)));
        m_driveController.rightTrigger(0.25).toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

    //manip cont bindings
    // m_manipController.x().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.NOTE)));
    // m_manipController.a().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.SPEAKER)));
    // m_manipController.b().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.AMP)));
    // m_manipController.y().onTrue(Commands.runOnce(()-> Aimlock.setDoState(DoState.SOURCE)));
    // new Trigger(()-> m_cameraSubsystem.isLLOdoGood(5.0)).whileTrue(m_rumble.setRumbleCommand(RumbleType.kBothRumble, 0.5, 0.25));
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

  // public ShooterSubsystem getShooter() {
  //   return m_shooter; //
  // }

  public Command getTeleOpCommand() {
    return driveCommand;
  }

  public void print() {
    this.m_swerve.print();
  }
}
