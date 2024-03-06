// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controller;
import frc.robot.Constants.ID;
import frc.robot.Subsystems.CameraSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.FeederMotorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Aimlock.DoState;
import frc.robot.Subsystems.PickupMotorsSubsystem;
import frc.robot.Subsystems.PickupOrchestrator;
import frc.robot.Subsystems.PneumaticsSubsystem;
import frc.robot.Commands.Drive;
import frc.robot.Commands.ForceShooterUpCommand;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.aimShooterCommand;
import frc.robot.Commands.ampAlignCommand;
import frc.robot.Commands.autoPickupNote;
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
  private final FeederMotorSubsystem m_feederMotor;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final LEDSubsystem m_LedSubsystem;
  private final PickupOrchestrator m_pickup;

  Command driveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
    this.m_cameraSubsystem = new CameraSubsystem();
    this.m_swerve = new Drivetrain(m_gyro, m_cameraSubsystem);
    this.m_pickupMotors = new PickupMotorsSubsystem();
    this.m_pneumatics = new PneumaticsSubsystem();
    this.m_feederMotor = new FeederMotorSubsystem();
    this.m_pickup = new PickupOrchestrator(m_pneumatics, m_pickupMotors, m_feederMotor);
    this.m_shooter = new ShooterSubsystem();
    this.m_aim = new Aimlock(m_swerve, m_shooter);
    this.m_elevatorSubsystem = new ElevatorSubsystem();
    this.m_LedSubsystem = new LEDSubsystem();

    m_swerve.setAim(m_aim);
    m_shooter.setAim(m_aim);

    // Xbox controllers return negative values when we push forward.
    driveCommand = new Drive(m_swerve);

    // Register named commands
    NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve)); // dont need anymore (?)
    NamedCommands.registerCommand("Lock Target in Auto",
        Commands.runOnce(() -> m_swerve.setLockTargetInAuto(true)));
    NamedCommands.registerCommand("Dont Lock Target in Auto",
        Commands.runOnce(() -> m_swerve.setLockTargetInAuto(false)));
    NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> {
      m_shooter.shoot();
      m_feederMotor.runShootFeederMotorToShoot();
    }));
    NamedCommands.registerCommand("Dont Shoot", Commands.runOnce(() -> {
      m_shooter.noShoot();
      m_feederMotor.stopFeederMotor();
    }));
    NamedCommands.registerCommand("Intake Down", m_pickup.runIntakeCommand());
    NamedCommands.registerCommand("Intake Up", m_pickup.disableIntakeCommand());
    NamedCommands.registerCommand("Pickup Note", new autoPickupNote(m_swerve));

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
    Controller.kManipulatorController.rightTrigger(0.1)
        .and(() -> Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece))
        .onTrue(Commands.runOnce(() -> {
          m_shooter.shoot();
          m_feederMotor.runShootFeederMotorToShoot();
        }));

    new Trigger(() -> m_shooter.hasShotNote()).onTrue(Commands.runOnce(() -> System.out.println("trigger worked")))
        .onTrue(new SequentialCommandGroup(
            new TimerWaitCommand(0.25),
            Commands.runOnce(() -> {
              m_shooter.noShoot();
            }),
            m_feederMotor.stopFeederMotorCommand()).finallyDo(() -> {
              Flags.pieceState = Flags.subsystemsStates.noPiece;
            }));

    Controller.kManipulatorController.back()
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.AMP)));
    Controller.kManipulatorController.start()
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.SPEAKER)));

    /* OVERRIDE CONTROLS BEGIN */

    // Manipulator "Y" button: Force the shooter to the up position and stop it
    // using the auto-aim feature as a toggle. If toggled off, resume using the
    // auto-aim feature
    // Motivation: The robot's auto-aim feature depends on the limelight camera and
    // for accurate vision from the field. If the limelight fails or the field is
    // unreliable, this will allow you to have a predictable shooter angle. We know
    // we can score with it all the way up and bumpered up to the speaker
    Controller.kManipulatorController.y()
        .toggleOnTrue(new ForceShooterUpCommand(m_shooter));

    // Manipulator "X" button: Force to loaded state and stop the feeder motors. Now
    // the robot thinks it has a piece.
    // Motivation: Can force the robot to think it has a piece loaded and ready to
    // shoot.
    Controller.kManipulatorController.x()
        .onTrue(Commands.runOnce(() -> Flags.pieceState = Flags.subsystemsStates.loadedPiece))
        .onTrue(m_feederMotor.stopFeederMotorCommand());

    // Manipulator "B" button: Signal the shooter to stop running, stop the feed
    // motors, and then transition to NoPiece state
    // Motivation: The robot failed to detect that it shot a piece. This can force
    // it back to a NoPiece state
    Controller.kManipulatorController.b()
        .onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> {
              m_shooter.noShoot();
            }),
            m_feederMotor.stopFeederMotorCommand())
            .finallyDo(() -> {
              Flags.pieceState = Flags.subsystemsStates.noPiece;
            }));

    // Manipulator "A" button: Run pickup in reverse while held. Let go: stop the
    // pickup motors. No state change!
    // Motivation: The piece got jammed. Run the pickup in reverse
    Controller.kManipulatorController.a()
        .whileTrue(Commands.run(() -> m_pickupMotors.runMotorsReverse(),
            m_pickupMotors))
        .onFalse(m_pickupMotors.stopMotorsCommand());

    /* OVERRIDE CONTROLS END */

    Controller.kDriveController.rightTrigger(0.1).and(m_pickup.noPieceTrigger)
        .whileTrue(m_pickup.runIntakeCommand())
        .onFalse(m_pickup.disableIntakeCommand());

    // while the intake is down and we hold the left trigger, autoPickupNote
    Controller.kDriveController.leftTrigger(0.1).and(() -> Aimlock.getNoteVision())
        .whileTrue(new autoPickupNote(m_swerve));
    // while the left trigger is held and we are in speaker mode, lock the speaker
    Controller.kDriveController.leftTrigger(0.1).and(() -> !Aimlock.getNoteVision())
        .and(() -> Aimlock.getDoState().equals(DoState.SPEAKER))
        .whileTrue(new lockTarget(m_swerve));
    // while the left trigger is held and we are in amp mode, go up to the amp
    Controller.kDriveController.leftTrigger(0.1).and(() -> !Aimlock.getNoteVision())
        .and(() -> Aimlock.getDoState().equals(DoState.AMP))
        .whileTrue(new RepeatCommand(new ampAlignCommand(m_swerve)));

    // drive cont bindings
    Controller.kDriveController.y().onTrue((new resetOdo(m_swerve)));
    Controller.kDriveController.rightBumper().onTrue(this.m_swerve.setFieldRelativeCommand(false))
        .onFalse(this.m_swerve.setFieldRelativeCommand(true));

    Controller.kDriveController.start().onTrue(Commands.runOnce(() -> Aimlock.setDoState(DoState.ENDGAME)));

    new Trigger(() -> m_shooter.getBottomLimit())
        .onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(0)).ignoringDisable(true));
    new Trigger(() -> m_shooter.getTopLimit()).onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(
        20)).ignoringDisable(true));

    // DEBUGGING MODE START
    Controller.kDriveController.povUp().whileTrue(Commands.run(() -> m_shooter.driveHoodManually(2.5)))
        .onFalse(Commands.run(() -> m_shooter.driveHoodManually(0)));
    Controller.kDriveController.povDown().whileTrue(Commands.run(() -> m_shooter.driveHoodManually(-2.5)))
        .onFalse(Commands.run(() -> m_shooter.driveHoodManually(0)));
    // DEBUGGING MODE END
  }

  Command auto1;
  Command pathfindAmp;

  private void configPathPlannerStuff() {
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // SmartDashboard.putData("Example Auto", AutoBuilder.buildAuto("Example
    // Auto"));
    // Add a button to run a simple example path

    auto1 = AutoBuilder.buildAuto("race auto");
    auto1.setName("AUTO1");
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

  public LEDSubsystem getLedSubsystem() {
    return m_LedSubsystem;
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
    return auto1;
  }

  public void setTeleDefaultCommand() {
    if (this.m_swerve.getDefaultCommand() == null) {
      this.m_swerve.setDefaultCommand(driveCommand);
    }
    if (this.m_shooter.getDefaultCommand() == null) {
      this.m_shooter.setDefaultCommand(new aimShooterCommand(m_shooter));
    }
  }

  public void setAutoDefaultCommand() {
    if (this.m_shooter.getDefaultCommand() == null) {
      this.m_shooter.setDefaultCommand(new aimShooterCommand(m_shooter));
    }
  }

  public void clearDefaultCommand() {
    this.m_swerve.removeDefaultCommand();
    this.m_shooter.removeDefaultCommand();
  }
}
