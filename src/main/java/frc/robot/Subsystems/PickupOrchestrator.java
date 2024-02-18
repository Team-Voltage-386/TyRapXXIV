package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.TimerWaitCommand;
import frc.robot.Constants.Controller;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class PickupOrchestrator extends SubsystemBase {
    private PneumaticsSubsystem m_pneumatics;
    private PickupMotorsSubsystem m_pickupMotors;

    // Temporary
    TalonSRX motorA = new TalonSRX(19);

    public PickupOrchestrator(PneumaticsSubsystem pneumatics, PickupMotorsSubsystem pickupMotors) {
        m_pickupMotors = pickupMotors;
        m_pneumatics = pneumatics;
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
        return m_pneumatics.disableLatchSolenoidCommand();
    }
}
