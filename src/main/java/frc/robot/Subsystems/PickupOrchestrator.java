package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

        // Automatically puts piece into the loaded position
        (lowSensorTrigger.negate().and(holdingPieceTrigger)).onTrue(loadPieceCommand())
                .onFalse(new SequentialCommandGroup(runOnce(() -> {
                    if (DriverStation.isEnabled()) {
                        Flags.pieceState = subsystemsStates.loadedPiece;
                    }
                })));
    }

    public SequentialCommandGroup runIntakeCommand() {
        return new SequentialCommandGroup(m_pneumatics.enableIntakeSolenoidCommand(),
                m_pickupMotors.runMotorsCommand());
    }

    public ParallelCommandGroup disableIntakeCommand() {
        return new ParallelCommandGroup(m_pneumatics.disableIntakeSolenoidCommand(),
                m_pickupMotors.runMotorsSlowCommand());
    }

    public Command loadPieceCommand() {
        return (m_FeederMotor.runFeederMotorToLoadCommand());
    }

    public Command stopLoadingPieceCommand() {
        return (m_FeederMotor.stopFeederMotorCommand());
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            if (Flags.pieceState.equals(subsystemsStates.noPiece)) {
                if (!lowHoodSensor.get()) {
                    (new SequentialCommandGroup(new TimerWaitCommand(0.25), disableIntakeCommand(),
                            runOnce(() -> {
                                if (DriverStation.isEnabled()) {
                                    Flags.pieceState = subsystemsStates.holdingPiece;
                                }
                            }))).schedule();
                }
            }
        }
    }
}
