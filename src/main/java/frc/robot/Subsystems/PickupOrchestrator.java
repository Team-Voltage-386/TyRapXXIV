package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TimerWaitCommand;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Flags.subsystemsStates;

public class PickupOrchestrator extends SubsystemBase {
    private PneumaticsSubsystem m_pneumatics;
    private PickupMotorsSubsystem m_pickupMotors;
    private FeederMotorSubsystem m_FeederMotor;

    // High hood sensor is not to be trusted until it is moved
    private DigitalInput highHoodSensor;
    private DigitalInput lowHoodSensor;

    public Trigger noPieceTrigger;
    public Trigger holdingPieceTrigger;
    public Trigger loadedPieceTrigger;
    public Trigger enabledTrigger;

    ShuffleboardTab IntakeSensors;

    SimpleWidget lowHoodSensorWidget;

    public PickupOrchestrator(PneumaticsSubsystem pneumatics, PickupMotorsSubsystem pickupMotors,
            FeederMotorSubsystem feederMotor) {
        m_pickupMotors = pickupMotors;
        m_pneumatics = pneumatics;
        m_FeederMotor = feederMotor;
        // highHoodSensor = new DigitalInput(6);
        lowHoodSensor = new DigitalInput(7);
        Trigger lowSensorTrigger = new Trigger(lowHoodSensor::get);
        // Trigger highSensorTrigger = new Trigger(highHoodSensor::get);
        noPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.noPiece));
        holdingPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.holdingPiece));
        enabledTrigger = new Trigger(() -> DriverStation.isEnabled());

        // Automatically puts piece into the loaded position
        (lowSensorTrigger.negate().and(holdingPieceTrigger)).onTrue(loadPieceCommand())
                .onFalse(new SequentialCommandGroup(runOnce(() -> {
                    if (DriverStation.isEnabled()) {
                        Flags.pieceState = subsystemsStates.loadedPiece;
                    }
                }), stopLoadingPieceCommand()));

        (lowSensorTrigger.negate()).and(noPieceTrigger).and(enabledTrigger)
                .onTrue(new SequentialCommandGroup(runOnce(() -> {
                    if (DriverStation.isEnabled()) {
                        Flags.pieceState = subsystemsStates.holdingPiece;
                    }
                }), disableIntakeCommand()));

        IntakeSensors = Shuffleboard.getTab("Intake");

        lowHoodSensorWidget = IntakeSensors.add("Low Hood Sensors", false);
    }

    public ParallelCommandGroup runIntakeCommand() {
        return new ParallelCommandGroup(m_pneumatics.enableIntakeSolenoidCommand(),
                m_pickupMotors.runMotorsCommand(), m_FeederMotor.runFeederMotorToLoadCommand());
    }

    public SequentialCommandGroup disableIntakeCommand() {
        return new SequentialCommandGroup(m_pneumatics.disableIntakeSolenoidCommand(),
                m_pickupMotors.runMotorsSlowCommand());
    }

    public InstantCommand loadPieceCommand() {
        return new InstantCommand(() -> m_FeederMotor.runFeederMotorToLoad());
    }

    public InstantCommand stopLoadingPieceCommand() {
        return new InstantCommand(() -> m_FeederMotor.stopFeederMotor());
    }

    @Override
    public void periodic() {
        lowHoodSensorWidget.getEntry().setBoolean(lowHoodSensor.get());

        // if (DriverStation.isEnabled()) {
        // if (Flags.pieceState.equals(subsystemsStates.noPiece)) {
        // if (!lowHoodSensor.get()) {
        // (new SequentialCommandGroup(runOnce(() -> {
        // if (DriverStation.isEnabled()) {
        // Flags.pieceState = subsystemsStates.holdingPiece;
        // }
        // }), disableIntakeCommand())).schedule();
        // }
        // }
        // }
    }
}
