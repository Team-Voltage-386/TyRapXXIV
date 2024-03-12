package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TrapSubsystem extends SubsystemBase {
    // Move these out of the subsystem into constants
    private int m_extendLimitDIOChannel = 3;
    private int kTrapExtendMotorID = 20;
    private int kTrapIntakeMotorID = 21;

    private int kTrapExtendMotorPDH = 4;
    private int kTrapIntakeMotorPDH = 12;

    private double kHoldingPieceCurrent = 10; // Figure out during testing. Should be the current drop when the intake
                                              // motor is holding a piece all the way back
    private double kMaxExtendInCurrentLimit = 7; // Found during testing. Measures the current spike when the trap
                                                 // extend motor is stalling as it tries to retract further in than the
                                                 // maximum physically allowed retract

    private PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);

    // Reflective sensor
    private DigitalInput m_extendLimit = new DigitalInput(m_extendLimitDIOChannel);

    // Stores the last direction the extend motor was moving for use in
    // increment/decrementTrapCounter
    // positive means going out, negative means going in
    private double m_extendMotorLastValue = 0;

    // Stores the position of the trap arm. 0 is at the bottom, 1 is source height,
    // 2 is max extend
    private int trapCounter = 0;

    private TalonSRX m_trapExtendMotor;
    private TalonSRX m_trapIntakeMotor;

    private ShuffleboardTab m_trapSubsystemTab;
    private SimpleWidget m_motorPercentageEntry;
    private SimpleWidget m_isLimitTriggeredEntry;
    private SimpleWidget m_trapCounterEntry;

    private ShuffleboardTab m_competitionTab;
    private SimpleWidget m_competitionTrapHeightEntry;

    public TrapSubsystem() {
        m_trapExtendMotor = new TalonSRX(kTrapExtendMotorID);
        m_trapExtendMotor.setInverted(true);
        m_trapIntakeMotor = new TalonSRX(kTrapIntakeMotorID);
        m_trapIntakeMotor.setInverted(true);

        m_trapSubsystemTab = Shuffleboard.getTab("Trap Subsystem");
        m_motorPercentageEntry = m_trapSubsystemTab.add("Extend Motor Current", 0.0);
        m_isLimitTriggeredEntry = m_trapSubsystemTab.add("Is Limit Triggered?", false);
        m_trapCounterEntry = m_trapSubsystemTab.add("Trap Counter", 0);

        m_competitionTab = Shuffleboard.getTab("Competition Tab");
        m_competitionTrapHeightEntry = m_competitionTab.add("Trap Max Extend", false).withSize(2, 1).withPosition(5,
                2);

        // This trigger handles the logic behind the trapCounter and the tape sensor.
        // Using the trigger allows you to single out of the edge of detection: when you
        // go from false to true or from true to false. The commands just hold some
        // logic to correctly increment/decrement the trap counter
        new Trigger(this::isLimitTriggered).onTrue(Commands.runOnce(() -> this.incrementTrapCounter()))
                .onFalse(Commands.runOnce(() -> this.decrementTrapCounter()));

    }

    public void setTrapExtendMotor(double motorPercentage) {
        m_trapExtendMotor.set(TalonSRXControlMode.PercentOutput, motorPercentage);
        m_extendMotorLastValue = motorPercentage;
    }

    public boolean isLimitTriggered() {
        // Returns the retroreflective tape sensor
        return !m_extendLimit.get();
    }

    public void setTrapIntakeMotor(double motorPercentage) {
        m_trapIntakeMotor.set(TalonSRXControlMode.PercentOutput, motorPercentage);
    }

    public double getIntakeMotorCurrent() {
        return m_PDH.getCurrent(kTrapIntakeMotorPDH);
    }

    public boolean isHoldingPiece() {
        return getIntakeMotorCurrent() < kHoldingPieceCurrent;
    }

    public double getExtendMotorCurrent() {
        return m_PDH.getCurrent(kTrapExtendMotorPDH);
    }

    public boolean isMaxRetract() {
        return getExtendMotorCurrent() > kMaxExtendInCurrentLimit;
    }

    public void setTrapIntakeMotorOn() {
        this.setTrapIntakeMotor(1);
    }

    public void setTrapIntakeMotorReverse() {
        this.setTrapIntakeMotor(-1);
    }

    public void setTrapIntakeMotorOff() {
        this.setTrapIntakeMotor(0.0);
    }

    public void resetTrapCounter() {
        trapCounter = 0;
    }

    public int getTrapCounter() {
        return trapCounter;
    }

    public void incrementTrapCounter() {
        // Put in the on true of the trigger. If you are going up and you see the tape
        // go from false to true, then you should increment the trap counter
        if (m_extendMotorLastValue > 0) {
            trapCounter++;
        }
        System.out.println("Trap Counter: " + trapCounter);
    }

    public void decrementTrapCounter() {
        // Put in the on false of the trigger. If you are going down and you see the
        // tape go from true to false, then you should decrement the trap counter
        if (m_extendMotorLastValue < 0) {
            trapCounter--;
        }
        System.out.println("Trap Counter: " + trapCounter);
    }

    @Override
    public void periodic() {
        m_motorPercentageEntry.getEntry().setDouble(getExtendMotorCurrent());
        m_isLimitTriggeredEntry.getEntry().setBoolean(isLimitTriggered());
        m_trapCounterEntry.getEntry().setInteger(trapCounter);

        m_competitionTrapHeightEntry.getEntry().setBoolean(trapCounter == 2);
    }

}
