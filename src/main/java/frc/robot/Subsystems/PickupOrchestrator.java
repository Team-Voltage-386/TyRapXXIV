package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
import frc.robot.Constants.Controller;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Flags.subsystemsStates;

import java.util.concurrent.locks.Condition;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class PickupOrchestrator extends SubsystemBase {
    private PneumaticsSubsystem m_pneumatics;
    private PickupMotorsSubsystem m_pickupMotors;
    private DigitalInput highHoodSensor;
    private DigitalInput lowHoodSensor;

    private SimpleWidget HoodDetected;
    private SimpleWidget LowDetected;
    // private SimpleWidget StateSensor;

    private ShuffleboardTab sensorTab;

    // Temporary
    TalonSRX motorA = new TalonSRX(19);

    public Trigger noPieceTrigger;
    public Trigger holdingPieceTrigger;
    public Trigger loadedPieceTrigger;

    public PickupOrchestrator(PneumaticsSubsystem pneumatics, PickupMotorsSubsystem pickupMotors) {
        m_pickupMotors = pickupMotors;
        m_pneumatics = pneumatics;
        // motorA.set(TalonSRXControlMode.PercentOutput, -0.2);
        highHoodSensor = new DigitalInput(6);
        lowHoodSensor = new DigitalInput(7);
        Trigger lowSensorTrigger = new Trigger(lowHoodSensor::get);
        Trigger highSensorTrigger = new Trigger(highHoodSensor::get);
        noPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.noPiece));
        holdingPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.holdingPiece));
        loadedPieceTrigger = new Trigger(() -> Flags.pieceState.equals(subsystemsStates.noPiece));

        // Automatically disables intake once piece is picked up
        (lowSensorTrigger.negate().and(highSensorTrigger.negate()).and(noPieceTrigger))
                .onTrue(new SequentialCommandGroup(disableIntakeCommand(),
                        runOnce(() -> Flags.pieceState = subsystemsStates.holdingPiece)));
        // Automatically puts piece into the loaded position
        (lowSensorTrigger.negate().and(highSensorTrigger.negate()).and(holdingPieceTrigger))
                .onTrue(new SequentialCommandGroup(loadPieceCommand(),
                        runOnce(() -> Flags.pieceState = subsystemsStates.loadedPiece)));
        // Detects no piece
        (lowSensorTrigger.and(noPieceTrigger.negate()))
                .onTrue(runOnce(() -> Flags.pieceState = subsystemsStates.noPiece));

        sensorTab = Shuffleboard.getTab("Sensors");
        HoodDetected = sensorTab.add("Hood Detected", false);
        LowDetected = sensorTab.add("Low Detected", false);

        // StateSensor = sensorTab.add("Piece State", false);
    }

    public SequentialCommandGroup runIntakeCommand() {
        return new SequentialCommandGroup(m_pneumatics.enableIntakeSolenoidCommand(),
                m_pickupMotors.runMotorsCommand());
    }

    public ParallelCommandGroup disableIntakeCommand() {
        return new ParallelCommandGroup(m_pneumatics.disableIntakeSolenoidCommand(),
                m_pickupMotors.stopMotorsCommand());
    }

    public SequentialCommandGroup loadPieceCommand() {
        return new SequentialCommandGroup(m_pneumatics.enableLoaderSolenoidCommand(), new TimerWaitCommand(0.25),
                m_pneumatics.enableLatchSolenoidCommand(), new TimerWaitCommand(0.25),
                m_pneumatics.disableLoaderSolenoidCommand());
    }

    public Command lowerLoaderCommand() {
        // Temporary
        Flags.pieceState = subsystemsStates.noPiece;
        return m_pneumatics.disableLatchSolenoidCommand();
    }

    @Override
    public void periodic() {
        HoodDetected.getEntry().setBoolean(highHoodSensor.get());
        LowDetected.getEntry().setBoolean(lowHoodSensor.get());
        // StateSensor.getEntry().setBoolean(Flags.pieceState.equals(Flags.subsystemsStates.noPiece));
    }
}
