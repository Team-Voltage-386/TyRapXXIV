// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.Subsystems.TrapSubsystem;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Flags;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.Aimlock.DoState;
import frc.robot.Subsystems.PickupMotorsSubsystem;
import frc.robot.Subsystems.PickupOrchestrator;
import frc.robot.Subsystems.PneumaticsSubsystem;
import frc.robot.Subsystems.RumbleSubsystem;
import frc.robot.Commands.AutoReadyLEDCommand;
import frc.robot.Commands.ContinuousRumble;
import frc.robot.Commands.Drive;
import frc.robot.Commands.ElevatorDownCommand;
import frc.robot.Commands.ElevatorUpCommand;
import frc.robot.Commands.EndgameModeCommand;
import frc.robot.Commands.ForceShooterUpCommand;
import frc.robot.Commands.IntakeDownLEDCommand;
import frc.robot.Commands.PieceObtainedAndAutoHasTargetLEDCommand;
import frc.robot.Commands.PieceObtainedAndAutoReadyLEDCommand;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.TargetAquiredLEDCommand;
import frc.robot.Commands.aimShooterCommand;
import frc.robot.Commands.autoPickupNote;
import frc.robot.Commands.lockTarget;
import frc.robot.Commands.TimerWaitCommand;
import frc.robot.Commands.TrapInCommand;
import frc.robot.Commands.TrapManualInCommand;
import frc.robot.Commands.TrapManualOutCommand;
import frc.robot.Commands.TrapOutCommand;
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
  private final SendableChooser<String> autoChooser;
  private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
  private final CameraSubsystem m_cameraSubsystem;
  private final Drivetrain m_swerve;
  private final ShooterSubsystem m_shooter;
  private final Aimlock m_aim;
  private final PickupMotorsSubsystem m_pickupMotors;
  private final PneumaticsSubsystem m_pneumatics;
  private final FeederMotorSubsystem m_feederMotor;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final TrapSubsystem m_trapSubsystem;
  private final RumbleSubsystem m_manipulatorRumbleSubsystem;
  private final RumbleSubsystem m_driverRumbleSubsystem;
  public final LEDSubsystem m_LedSubsystem;
  private final PickupOrchestrator m_pickup;
  public static Trigger aimWithingErrorBounds;
  public static Trigger validLimelightTrigger;

  private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");

  Command driveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.m_manipulatorRumbleSubsystem = new RumbleSubsystem(Controller.kManipulatorController);
    this.m_driverRumbleSubsystem = new RumbleSubsystem(Controller.kDriveController);
    this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
    this.m_cameraSubsystem = new CameraSubsystem();
    this.m_LedSubsystem = new LEDSubsystem();
    this.m_swerve = new Drivetrain(m_gyro, m_cameraSubsystem);
    this.m_pickupMotors = new PickupMotorsSubsystem();
    this.m_pneumatics = new PneumaticsSubsystem();
    this.m_feederMotor = new FeederMotorSubsystem();
    this.m_pickup = new PickupOrchestrator(m_pneumatics, m_pickupMotors, m_feederMotor, m_driverRumbleSubsystem,
        m_LedSubsystem);
    this.m_shooter = new ShooterSubsystem();
    this.m_aim = new Aimlock(m_swerve, m_shooter);
    this.m_trapSubsystem = new TrapSubsystem(m_manipulatorRumbleSubsystem);
    this.m_elevatorSubsystem = new ElevatorSubsystem(m_LedSubsystem, m_manipulatorRumbleSubsystem);

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
    NamedCommands.registerCommand("I SHOT.", Commands.runOnce(() -> Flags.pieceState = Flags.subsystemsStates.noPiece));
    NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> {
      m_shooter.shoot();
      m_feederMotor.runFeederMotorToShoot();
    }));
    NamedCommands.registerCommand("Dont Shoot", Commands.runOnce(() -> {
      m_shooter.noShoot();
      m_feederMotor.stopFeederMotor();
    }));
    NamedCommands.registerCommand("rapidfire", Commands.runOnce(m_feederMotor::enableRapidFire));
    NamedCommands.registerCommand("norapidfire", Commands.runOnce(m_feederMotor::disableRapidFire));
    NamedCommands.registerCommand("Intake Down", m_pickup.runIntakeCommand());
    NamedCommands.registerCommand("Intake Up", m_pickup.disableIntakeCommand());
    NamedCommands.registerCommand("Pickup Note", new autoPickupNote(m_swerve));

    autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'
    // Create choices for autonomous functions in the Smart Dashboard
    configPathPlannerStuff();
    configureBindings();
    autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
    m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);
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
    // New Triggers
    Trigger endgameButtons = new Trigger(() -> Flags.buttonMapMode == Flags.buttonMapStates.endgameMode);

    validLimelightTrigger = new Trigger(() -> LimelightHelpers.getTV("limelight-b"));

    aimWithingErrorBounds = new Trigger(
        () -> Math.abs(Math.abs(Math.toRadians(m_gyro.getYaw().getValueAsDouble()))
            - Math.abs(m_aim.getSpeakerAimTargetAngle())) < Math
                .toRadians(10));

    aimWithingErrorBounds.and(m_pickup.noPieceTrigger.negate())
        .whileTrue(new PieceObtainedAndAutoReadyLEDCommand(m_LedSubsystem));

    aimWithingErrorBounds.and(m_pickup.noPieceTrigger).whileTrue(new AutoReadyLEDCommand(m_LedSubsystem));

    validLimelightTrigger.and(m_pickup.loadedPieceTrigger)
        .whileTrue(new TargetAquiredLEDCommand(m_LedSubsystem));

    validLimelightTrigger.and(m_pickup.noPieceTrigger.negate())
        .whileTrue(new PieceObtainedAndAutoHasTargetLEDCommand(m_LedSubsystem));

    Controller.kManipulatorController.leftStick().and(endgameButtons.negate())
        .onFalse(new SequentialCommandGroup(
            Commands.runOnce(() -> Flags.buttonMapMode = Flags.buttonMapStates.endgameMode),
            Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.ENDGAME)),
            new EndgameModeCommand(m_LedSubsystem),
            Commands.runOnce(() -> System.out.println("endgame mode activated"))));

    Controller.kManipulatorController.leftStick().and(endgameButtons)
        .onFalse(new SequentialCommandGroup(
            Commands.runOnce(() -> m_elevatorSubsystem.setElevatorMotorsVoltage(0)),
            Commands.runOnce(() -> Flags.buttonMapMode = Flags.buttonMapStates.notEndgameMode),
            Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.SPEAKER)),
            Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorOff()),
            Commands.runOnce(() -> System.out.println("endgame mode disable"))));

    // Endgame buttons
    Controller.kManipulatorController.povUp().and(endgameButtons)
        .onTrue(new TrapOutCommand(m_trapSubsystem));

    Controller.kManipulatorController.povDown().and(endgameButtons)
        .onTrue(new TrapInCommand(m_trapSubsystem));

    Controller.kManipulatorController.povRight().and(endgameButtons)
        .whileTrue(new TrapManualOutCommand(m_trapSubsystem));

    Controller.kManipulatorController.povLeft().and(endgameButtons)
        .whileTrue(new TrapManualInCommand(m_trapSubsystem));

    Controller.kManipulatorController.rightBumper().and(endgameButtons)
        .whileTrue(new ElevatorUpCommand(m_elevatorSubsystem, m_manipulatorRumbleSubsystem, m_LedSubsystem));

    Controller.kManipulatorController.leftBumper().and(endgameButtons)
        .whileTrue(new ElevatorDownCommand(m_elevatorSubsystem, m_manipulatorRumbleSubsystem, m_LedSubsystem));

    Controller.kManipulatorController.start().and(endgameButtons)
        .onTrue(m_pneumatics.disableLegSolenoidCommand());

    Controller.kManipulatorController.back().and(endgameButtons)
        .onTrue(m_pneumatics.enableLegSolenoidCommand());

    Controller.kManipulatorController.axisGreaterThan(3, 0.25).and(endgameButtons)
        .onTrue(Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorOn()));

    Controller.kManipulatorController.axisGreaterThan(2, 0.25).and(endgameButtons)
        .onTrue(Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorReverse()));

    // Regular non endgame button controlls
    Controller.kManipulatorController.rightTrigger(0.1).and(endgameButtons.negate())
        .and(() -> Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece))
        .onTrue(Commands.runOnce(() -> {
          m_shooter.shoot();
          m_feederMotor.runFeederMotorToShoot();
        }));

    new Trigger(() -> m_shooter.hasShotNote()).onTrue(Commands.runOnce(() -> System.out.println("Shot Note.")))
        .onTrue(new SequentialCommandGroup(
            new TimerWaitCommand(0.25),
            Commands.runOnce(() -> {
              m_shooter.noShoot();
            }),
            m_feederMotor.stopFeederMotorCommand()).finallyDo(() -> {
              Flags.pieceState = Flags.subsystemsStates.noPiece;
            }));

    Controller.kManipulatorController.back().and(endgameButtons.negate())
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.AMP)));
    Controller.kManipulatorController.start().and(endgameButtons.negate())
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.SPEAKER)));
    Controller.kManipulatorController.leftTrigger(0.1).and(endgameButtons.negate())
        .onTrue(Commands.runOnce(() -> Aimlock.setDoState(Aimlock.DoState.PASS)));

    /* OVERRIDE CONTROLS BEGIN */

    // Manipulator "Y" button: Force the shooter to the up position and stop it
    // using the auto-aim feature as a toggle. If toggled off, resume using the
    // auto-aim feature
    // Motivation: The robot's auto-aim feature depends on the limelight camera and
    // for accurate vision from the field. If the limelight fails or the field is
    // unreliable, this will allow you to have a predictable shooter angle. We know
    // we can score with it all the way up and bumpered up to the speaker
    Controller.kManipulatorController.y().and(endgameButtons.negate())
        .toggleOnTrue(new ForceShooterUpCommand(m_shooter));

    // Manipulator "X" button: Force to loaded state and stop the feeder motors. Now
    // the robot thinks it has a piece.
    // Motivation: Can force the robot to think it has a piece loaded and ready to
    // shoot.
    Controller.kManipulatorController.x().and(endgameButtons.negate())
        .onTrue(Commands.runOnce(() -> Flags.pieceState = Flags.subsystemsStates.loadedPiece))
        .onTrue(m_feederMotor.stopFeederMotorCommand());

    // Manipulator "B" button: Signal the shooter to stop running, stop the feed
    // motors, and then transition to NoPiece state
    // Motivation: The robot failed to detect that it shot a piece. This can force
    // it back to a NoPiece state
    Controller.kManipulatorController.b().and(endgameButtons.negate())
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
    Controller.kManipulatorController.a().and(endgameButtons.negate())
        .whileTrue(Commands.run(() -> m_pickupMotors.runMotorsReverse(),
            m_pickupMotors))
        .onFalse(m_pickupMotors.stopMotorsCommand());

    /* OVERRIDE CONTROLS END */

    Controller.kDriveController.rightTrigger(0.1).and(m_pickup.noPieceTrigger).and(endgameButtons.negate())
        .whileTrue(new ParallelCommandGroup(m_pickup.runIntakeCommand(), new IntakeDownLEDCommand(m_LedSubsystem)))
        .onFalse(m_pickup.disableIntakeCommand());

    // while the intake is down and we hold the left trigger, autoPickupNote
    Controller.kDriveController.leftTrigger(0.1).and(() -> Aimlock.getNoteVision()).and(endgameButtons.negate())
        .whileTrue(new autoPickupNote(m_swerve));
    // while the left trigger is held and we are in speaker mode, lock the speaker
    Controller.kDriveController.leftTrigger(0.1).and(() -> !Aimlock.getNoteVision()).and(endgameButtons.negate())
        .and(() -> Aimlock.getDoState().equals(DoState.SPEAKER)).whileTrue(
            new ParallelCommandGroup(new lockTarget(m_swerve),
                new ContinuousRumble(m_manipulatorRumbleSubsystem, 0.05)));

    // drive cont bindings
    Controller.kDriveController.y().onTrue((new resetOdo(m_swerve)));
    Controller.kDriveController.rightBumper()
        .onTrue(this.m_swerve.setFieldRelativeCommand(false))
        .onFalse(this.m_swerve.setFieldRelativeCommand(true));

    new Trigger(() -> m_shooter.getBottomLimit())
        .onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(0)).ignoringDisable(true));
    new Trigger(() -> m_shooter.getTopLimit())
        .onTrue(Commands.runOnce(() -> m_shooter.setRelativeShooterEncoder(20)).ignoringDisable(true));

    // DEBUGGING MODE START
    Controller.kDriveController.povUp().whileTrue(Commands.run(() -> m_shooter.driveHoodManually(2.5)))
        .onFalse(Commands.run(() -> m_shooter.driveHoodManually(0)));
    Controller.kDriveController.povDown().whileTrue(Commands.run(() -> m_shooter.driveHoodManually(-2.5)))
        .onFalse(Commands.run(() -> m_shooter.driveHoodManually(0)));
    // DEBUGGING MODE END

    Controller.kDriveController.leftBumper().onTrue(m_swerve.setDriveMultCommand(0.5))
        .onFalse(m_swerve.setDriveMultCommand(1));
  }

  Command pathfindAmp;

  private void configPathPlannerStuff() {
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // SmartDashboard.putData("Example Auto", AutoBuilder.buildAuto("Example
    // Auto"));
    // Add a button to run a simple example path

    autoChooser.addOption("4 Piece B", "4 piece B");
    autoChooser.addOption("4.5 Piece B", "4.5 piece B");
    autoChooser.addOption("5 Piece (B4)", "5 piece (B4)");
    autoChooser.addOption("5 Piece (B4) v2", "5 piece (B4) v2");
    autoChooser.addOption("Shoot & Pickup", "shoot and backup");
    autoChooser.addOption("Shoot & Do Nothing", "shoot and do nothing");
    autoChooser.addOption("Race Auto", "race auto B");
    autoChooser.addOption("4 pce Race Auto", "4pce race auto B");
    autoChooser.addOption("Robonaut Race", "Robonaut Race");
    autoChooser.addOption("bugtest", "bugtestauto");

    // auto1.setName("AUTO1");

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

  public FeederMotorSubsystem getFeederMotor() {
    return m_feederMotor;
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
    if (autoChooser.getSelected().equals("NO AUTO")) {
      return Commands.none();
    }
    return AutoBuilder.buildAuto(autoChooser.getSelected());
  }

  public void setTeleDefaultCommand() {
    if (this.m_swerve.getDefaultCommand() == null) {
      this.m_swerve.setDefaultCommand(new ParallelCommandGroup(driveCommand,
          new RepeatCommand(new SequentialCommandGroup(new TimerWaitCommand(3),
              Commands.runOnce(m_swerve::fixOdo)))));
      // this.m_swerve.setDefaultCommand(driveCommand);
      // every 5 seconds fix the odometry
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
