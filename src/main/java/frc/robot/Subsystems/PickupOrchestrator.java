package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DoublePulseRumble;
import frc.robot.Commands.IntakeDownLEDCommand;
import frc.robot.Commands.PieceObtainedLEDCommand;
import frc.robot.Commands.SinglePulseRumble;
import frc.robot.Commands.TimerWaitCommand;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Flags.buttonMapStates;
import frc.robot.Utils.Flags.subsystemsStates;

public class PickupOrchestrator extends SubsystemBase {
    private PneumaticsSubsystem m_pneumatics;
    private PickupMotorsSubsystem m_pickupMotors;
    private FeederMotorSubsystem m_FeederMotor;
    private RumbleSubsystem m_driveRumble;
    private LEDSubsystem m_LEDSubsystem;

    private DigitalInput rightHoodSensor;
    private DigitalInput leftHoodSensor;

    public Trigger noPieceTrigger;
    public Trigger holdingPieceTrigger;
    public Trigger loadedPieceTrigger;
    public Trigger enabledTrigger;
    public Trigger AutoTrigger;
    public Trigger endgameTime;
    public Trigger alwaysShootingTrigger;

    ShuffleboardTab IntakeSensors;

    SimpleWidget leftHoodSensorWidget;
    SimpleWidget HoldingPieceWidget;
    SimpleWidget rightHoodSensorWidget;

    ShuffleboardTab m_competitionTab;
    SimpleWidget m_competitionLoadedPieceEntry;
    SimpleWidget m_competitionIntakeDownEntry;

    private boolean isIntakeDown = false;

    public PickupOrchestrator(PneumaticsSubsystem pneumatics, PickupMotorsSubsystem pickupMotors,
            FeederMotorSubsystem feederMotor, RumbleSubsystem driveRumble, LEDSubsystem ledSubsystem) {
        m_LEDSubsystem = ledSubsystem;
        m_pickupMotors = pickupMotors;
        m_pneumatics = pneumatics;
        m_FeederMotor = feederMotor;
        m_driveRumble = driveRumble;
        rightHoodSensor = new DigitalInput(6);
        leftHoodSensor = new DigitalInput(7);
        Trigger leftSensorTrigger = new Trigger(leftHoodSensor::get);
        Trigger rightSensorTrigger = new Trigger(rightHoodSensor::get);
        noPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.noPiece));
        holdingPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.holdingPiece));
        endgameTime = new Trigger(() -> Flags.buttonMapMode == Flags.buttonMapStates.endgameMode);
        enabledTrigger = new Trigger(() -> DriverStation.isEnabled());
        AutoTrigger = new Trigger(() -> DriverStation.isAutonomousEnabled());
        alwaysShootingTrigger = new Trigger(() -> m_FeederMotor.getRapidFire());

        // Automatically puts piece into the loaded position
        (holdingPieceTrigger.and(endgameTime.negate()).and((leftSensorTrigger.or(rightSensorTrigger)))
                .and(alwaysShootingTrigger.negate())) // when im not trying to rapid fire
                .onTrue(stopLoadingPieceCommand().withName("CHANGE_TO_LOADED_PIECE_S_STOP_LOADING").finallyDo(
                        () -> {
                            if (DriverStation.isEnabled()) {
                                Flags.pieceState = subsystemsStates.loadedPiece;
                            }
                        }));

        (leftSensorTrigger.negate().or(rightSensorTrigger.negate())).and(endgameTime.negate()).and(noPieceTrigger)
                .and(enabledTrigger)
                .and(alwaysShootingTrigger.negate()) // when im not trying to rapid fire
                .onTrue(new ParallelCommandGroup(
                        new DoublePulseRumble(new SinglePulseRumble(m_driveRumble, 0.75, 0.4), 0.3),
                        new SequentialCommandGroup(
                                disableIntakeCommand(),
                                loadPieceCommand().withName("LOAD PIECE"))
                                .withName("CHANGE_TO_HOLDING_PIECE_S_DISABLE_INTAKE")
                                .finallyDo(() -> {
                                    if (DriverStation.isEnabled()) {
                                        Flags.pieceState = subsystemsStates.holdingPiece;
                                        new PieceObtainedLEDCommand(ledSubsystem).schedule();
                                    }
                                })));

        AutoTrigger.and(noPieceTrigger).onTrue(runIntakeCommand().alongWith(new IntakeDownLEDCommand(ledSubsystem)));

        AutoTrigger.and(alwaysShootingTrigger) // rapid fire. go dumb and shoot pieces thru the robot, if we touch it,
                                               // it shoots it. only for auto.
                .onTrue(runIntakeForRapidFireCommand().alongWith(new IntakeDownLEDCommand(ledSubsystem)));

        alwaysShootingTrigger.onFalse(m_FeederMotor.runFeederMotorToLoadCommand());

        IntakeSensors = Shuffleboard.getTab("Intake");

        leftHoodSensorWidget = IntakeSensors.add("Left Hood Sensors", false);
        rightHoodSensorWidget = IntakeSensors.add("Right Hood Sensors", false);
        HoldingPieceWidget = IntakeSensors.add("Loaded Piece", false);

        m_competitionTab = Shuffleboard.getTab("Competition Tab");
        m_competitionLoadedPieceEntry = m_competitionTab.add("Loaded Piece", false).withSize(2, 1).withPosition(5, 3);
        m_competitionIntakeDownEntry = m_competitionTab.add("Intake Down", false).withSize(2, 1).withPosition(7, 1);
    }

    public ParallelCommandGroup runIntakeCommand() {
        return new ParallelCommandGroup(Commands.runOnce(() -> Aimlock.setNoteVision(true)),
                m_pneumatics.enableIntakeSolenoidCommand(), Commands.runOnce(() -> isIntakeDown = true),
                m_pickupMotors.runMotorsCommand(), m_FeederMotor.runFeederMotorToLoadCommand());
    }

    public ParallelCommandGroup runIntakeForRapidFireCommand() {
        return new ParallelCommandGroup(Commands.runOnce(() -> Aimlock.setNoteVision(true)),
                m_pneumatics.enableIntakeSolenoidCommand(), Commands.runOnce(() -> isIntakeDown = true),
                m_pickupMotors.runMotorsCommand(), m_FeederMotor.runFeederMotorToShootCommand());
    }

    public Command disableIntakeCommand() {
        return new SequentialCommandGroup(Commands.runOnce(() -> Aimlock.setNoteVision(false)),
                new TimerWaitCommand(0.25), m_pneumatics.disableIntakeSolenoidCommand(),
                Commands.runOnce(() -> isIntakeDown = false),
                m_pickupMotors.runMotorsSlowCommand())
                .withName("DISABLE_INTAKE_COMMAND");
    }

    public Command loadPieceCommand() {
        return m_FeederMotor.runFeederMotorToLoadCommand();
    }

    public Command stopLoadingPieceCommand() {
        return m_FeederMotor.stopFeederMotorCommand();
    }

    @Override
    public void periodic() {
        rightHoodSensorWidget.getEntry().setBoolean(rightHoodSensor.get());
        leftHoodSensorWidget.getEntry().setBoolean(leftHoodSensor.get());
        HoldingPieceWidget.getEntry().setBoolean(Flags.pieceState.equals(subsystemsStates.loadedPiece));

        m_competitionLoadedPieceEntry.getEntry().setBoolean(Flags.pieceState.equals(subsystemsStates.loadedPiece));
        m_competitionIntakeDownEntry.getEntry().setBoolean(isIntakeDown);
    }
}
