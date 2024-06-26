package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ClimbLimitLEDCommand;
import frc.robot.Commands.DoublePulseRumble;
import frc.robot.Commands.SinglePulseRumble;

public class ElevatorSubsystem extends SubsystemBase {

    // Move these out of the subsystem into constants
    private int kElevatorMotor1ID = 20;
    private int kElevatorMotor2ID = 21;
    private int kElevatorUpperLimitDIOChannel = 4;
    private int kElevatorLowerLimitDIOChannel = 5;
    private int kElevatorHandoffLimitDIOChannel = 2;

    // magnetic sensor
    private DigitalInput m_elevatorUpperLimit;
    // limit switch
    private DigitalInput m_elevatorLowerLimit;
    // limit switch for handoff
    private DigitalInput m_elevatorHandoffLimit;

    private CANSparkMax m_elevatorMotor1;
    private CANSparkMax m_elevatorMotor2;

    private ShuffleboardTab m_elevatorSubsystemTab;
    private SimpleWidget m_motorVoltageEntry;
    private SimpleWidget m_isUpperLimitTriggeredEntry;
    private SimpleWidget m_isLowerLimitTriggeredEntry;

    private ShuffleboardTab m_competitionTab;
    private SimpleWidget m_competitionIsUpperLimitTriggeredEntry;
    private SimpleWidget m_competitionIsLowerLimitTriggeredEntry;
    private SimpleWidget m_competitionElevatorHandoffTriggeredEntry;

    private double lastVoltageSet;

    LEDSubsystem m_LedSubsystem;
    RumbleSubsystem m_manipulatorRumble;

    Trigger middleMagTrigger;

    public ElevatorSubsystem(LEDSubsystem ledSubsystem, RumbleSubsystem manipRumble) {
        m_LedSubsystem = ledSubsystem;
        m_manipulatorRumble = manipRumble;

        m_elevatorMotor1 = new CANSparkMax(kElevatorMotor1ID, MotorType.kBrushless);
        m_elevatorMotor2 = new CANSparkMax(kElevatorMotor2ID, MotorType.kBrushless);

        m_elevatorUpperLimit = new DigitalInput(kElevatorUpperLimitDIOChannel);
        m_elevatorLowerLimit = new DigitalInput(kElevatorLowerLimitDIOChannel);
        m_elevatorHandoffLimit = new DigitalInput(kElevatorHandoffLimitDIOChannel);

        m_elevatorMotor1.setInverted(true);
        m_elevatorMotor2.setInverted(false); // It's following Motor1 anyways

        m_elevatorMotor1.setIdleMode(IdleMode.kBrake);
        m_elevatorMotor2.setIdleMode(IdleMode.kBrake);

        m_elevatorMotor2.follow(m_elevatorMotor1);

        m_elevatorSubsystemTab = Shuffleboard.getTab("Elevator Subsystem");
        m_motorVoltageEntry = m_elevatorSubsystemTab.add("Motor Voltage", 0.0).withSize(2, 1).withPosition(0, 1);
        m_isUpperLimitTriggeredEntry = m_elevatorSubsystemTab.add("Is Upper Limit Triggered?", false).withSize(2, 1)
                .withPosition(0, 2);
        m_isLowerLimitTriggeredEntry = m_elevatorSubsystemTab.add("Is Lower Limit Triggered", false).withSize(2, 1)
                .withPosition(0, 3);

        m_competitionTab = Shuffleboard.getTab("Competition Tab");
        m_competitionIsLowerLimitTriggeredEntry = m_competitionTab.add("Elevator Upper Limit", false).withSize(2, 1)
                .withPosition(5, 0);
        m_competitionIsUpperLimitTriggeredEntry = m_competitionTab.add("Elevator Lower Limit", false).withSize(2, 1)
                .withPosition(5, 1);
        m_competitionElevatorHandoffTriggeredEntry = m_competitionTab.add("Elevator Handoff", false).withSize(2, 1)
                .withPosition(5, 2);

        middleMagTrigger = new Trigger(() -> this.isHandoffLimitTriggered());
        middleMagTrigger.onTrue(
                new SinglePulseRumble(m_manipulatorRumble, 1.0, 0.5));
    }

    public void setElevatorMotorsVoltage(double motorVoltage) {
        lastVoltageSet = motorVoltage;
        m_elevatorMotor1.setVoltage(motorVoltage);
    }

    public boolean isUpperLimitTriggered() {
        return !m_elevatorUpperLimit.get(); // need to reverse because DIO returns backwards
    }

    public boolean isLowerLimitTriggered() {
        return !m_elevatorLowerLimit.get(); // need to reverse becuase DIO returns backwards
    }

    public boolean isHandoffLimitTriggered() {
        return !m_elevatorHandoffLimit.get();
    }

    public void updateShuffleboardWidgets() {
        m_motorVoltageEntry.getEntry().setDouble(lastVoltageSet);
        m_isUpperLimitTriggeredEntry.getEntry().setBoolean(isUpperLimitTriggered());
        m_isLowerLimitTriggeredEntry.getEntry().setBoolean(isLowerLimitTriggered());

        m_competitionIsLowerLimitTriggeredEntry.getEntry().setBoolean(isLowerLimitTriggered());
        m_competitionIsLowerLimitTriggeredEntry.getEntry().setBoolean(isUpperLimitTriggered());
        m_competitionElevatorHandoffTriggeredEntry.getEntry().setBoolean(isHandoffLimitTriggered());
    }

    @Override
    public void periodic() {
        updateShuffleboardWidgets();
    }

}
