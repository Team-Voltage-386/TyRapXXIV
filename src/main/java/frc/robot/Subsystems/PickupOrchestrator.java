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
import frc.robot.Commands.TimerWaitCommand;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Flags.buttonMapStates;
import frc.robot.Utils.Flags.subsystemsStates;

public class PickupOrchestrator extends SubsystemBase {
    private PneumaticsSubsystem m_pneumatics;
    private PickupMotorsSubsystem m_pickupMotors;
    private FeederMotorSubsystem m_FeederMotor;

    private DigitalInput rightHoodSensor;
    private DigitalInput leftHoodSensor;

    public Trigger noPieceTrigger;
    public Trigger holdingPieceTrigger;
    public Trigger loadedPieceTrigger;
    public Trigger enabledTrigger;
    public Trigger AutoTrigger;

    ShuffleboardTab IntakeSensors;

    SimpleWidget leftHoodSensorWidget;
    SimpleWidget HoldingPieceWidget;
    SimpleWidget rightHoodSensorWidget;

    public PickupOrchestrator(PneumaticsSubsystem pneumatics, PickupMotorsSubsystem pickupMotors,
            FeederMotorSubsystem feederMotor) {
        m_pickupMotors = pickupMotors;
        m_pneumatics = pneumatics;
        m_FeederMotor = feederMotor;
        rightHoodSensor = new DigitalInput(6);
        leftHoodSensor = new DigitalInput(7);
        Trigger leftSensorTrigger = new Trigger(leftHoodSensor::get);
        Trigger rightSensorTrigger = new Trigger(rightHoodSensor::get);
        noPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.noPiece));
        holdingPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.holdingPiece));
        enabledTrigger = new Trigger(() -> DriverStation.isEnabled());
        AutoTrigger = new Trigger(() -> DriverStation.isAutonomousEnabled());

        // Automatically puts piece into the loaded position
        (holdingPieceTrigger.and((leftSensorTrigger.or(rightSensorTrigger))))
                .onTrue(stopLoadingPieceCommand().withName("CHANGE_TO_LOADED_PIECE_S_STOP_LOADING").finallyDo(
                        () -> {
                            if (DriverStation.isEnabled()) {
                                Flags.pieceState = subsystemsStates.loadedPiece;
                            }
                        }));

        (leftSensorTrigger.negate().or(rightSensorTrigger.negate())).and(noPieceTrigger).and(enabledTrigger)
                .onTrue(new SequentialCommandGroup(
                        disableIntakeCommand(),
                        loadPieceCommand().withName("LOAD PIECE")).withName("CHANGE_TO_HOLDING_PIECE_S_DISABLE_INTAKE")
                        .finallyDo(() -> {
                            if (DriverStation.isEnabled()) {
                                Flags.pieceState = subsystemsStates.holdingPiece;
                            }
                        }));

        AutoTrigger.and(noPieceTrigger).onTrue(runIntakeCommand());

        IntakeSensors = Shuffleboard.getTab("Intake");

        leftHoodSensorWidget = IntakeSensors.add("Left Hood Sensors", false);
        rightHoodSensorWidget = IntakeSensors.add("Right Hood Sensors", false);
        HoldingPieceWidget = IntakeSensors.add("Loaded Piece", false);
    }

    public ParallelCommandGroup runIntakeCommand() {
        return new ParallelCommandGroup(Commands.runOnce(() -> Aimlock.setNoteVision(true)),
                m_pneumatics.enableIntakeSolenoidCommand(),
                m_pickupMotors.runMotorsCommand(), m_FeederMotor.runFeederMotorToLoadCommand());
    }

    public Command disableIntakeCommand() {
        return new SequentialCommandGroup(Commands.runOnce(() -> Aimlock.setNoteVision(false)),
                new TimerWaitCommand(0.25), m_pneumatics.disableIntakeSolenoidCommand(),
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
    }
}
