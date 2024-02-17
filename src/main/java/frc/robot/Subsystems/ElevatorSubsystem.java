package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ID;

public class ElevatorSubsystem extends SubsystemBase {

    public static enum ElevatorDirectionOption {
        UP,
        DOWN
    }

    private DigitalInput m_highSwitch;
    private DigitalInput m_lowSwitch;

    private CANSparkMax m_motorA;
    private CANSparkMax m_motorB;

    private ShuffleboardTab elevatorTab;
    private SimpleWidget m_topLimitWidget;
    private SimpleWidget m_lowLimitWidget;
    private SimpleWidget m_motorVoletWidget;

    public ElevatorSubsystem() {
        elevatorTab = Shuffleboard.getTab("Elevator Subsystem");

        m_highSwitch = new DigitalInput(3);
        m_lowSwitch = new DigitalInput(4);

        m_motorA = new CANSparkMax(ID.kElevatorMotorAID, MotorType.kBrushless);
        m_motorB = new CANSparkMax(ID.kElevatorMotorBID, MotorType.kBrushless);

    }

    @Override
    public void periodic() {

    }
}
