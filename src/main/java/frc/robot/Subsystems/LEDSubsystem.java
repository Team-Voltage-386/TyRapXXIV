package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.AutoReadyLEDCommand;
import frc.robot.Commands.EndgameModeCommand;
import frc.robot.Constants.LEDs;

public class LEDSubsystem extends SubsystemBase {

    // Addresable LED
    private AddressableLED m_led = new AddressableLED(LEDs.kpwm);

    // The LED Buffer
    // Reuse buffer
    // start empty output
    // Length is expensive to set, so only set it once, then just update data
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDs.kLEDBufferLength);

    private EndgameModeCommand m_EndgameModeLEDCommand;
    private AutoReadyLEDCommand m_AutoReadyLEDCommand;

    public LEDSubsystem() { // Constructor
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        m_EndgameModeLEDCommand = new EndgameModeCommand(this);
        m_AutoReadyLEDCommand = new AutoReadyLEDCommand(this);
    }

    public void updateLEDs() { // Updates the LEDs
        m_led.setData(m_ledBuffer);
    }

    public void clearExteriorSegmants() {
        for (int i = 0; i < m_ledBuffer.getLength() / 4; i++) {
            // While i is smaller than 1/4th of the LEDs, turn them off.
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        // Math.round((m_ledBuffer.getLength() / 4) * 3)
        for (int i = 33; i < m_ledBuffer.getLength(); i++) {
            // While i is bigger than 3/4ths of the LEDs, turn them off.
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        updateLEDs();
    }

    public void clearInteriorSegmant() {
        for (int i = m_ledBuffer.getLength() / 2 + m_ledBuffer.getLength() / 4; i > m_ledBuffer.getLength() / 4; i--) {
            // While i is between 1/4th and 3/4ths of the total LED Count, turn them off.
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        updateLEDs();
    }

    public void setExteriorSegmants(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength() / 4; i++) {
            // While i is smaller than 1/4th of the LEDs, Set them to specified color.
            m_ledBuffer.setRGB(i, r, g, b);
        }

        for (int i = (m_ledBuffer.getLength() - (m_ledBuffer.getLength() / 4)); i < m_ledBuffer.getLength(); i++) {
            // While i is bigger than 3/4ths of the LEDs, Set them to specified color.
            m_ledBuffer.setRGB(i, r, g, b);
        }
        updateLEDs();
    }

    public void setInteriorSegmant(int r, int g, int b) {
        for (int i = m_ledBuffer.getLength() / 4; i < m_ledBuffer.getLength() * 3 / 4; i++) {
            // While i is between 1/4th and 3/4ths of the total LED Count, Set them to
            // specified color.
            m_ledBuffer.setRGB(i, r, g, b);
        }
        updateLEDs();
    }

    public Command getEndgameModeCommand() {
        return this.m_EndgameModeLEDCommand;
    }

    public Command getAutoReadyCommand() {
        return this.m_AutoReadyLEDCommand;
    }

    public void setLedColor(int index, int r, int g, int b) {
        m_ledBuffer.setRGB(index, r, g, b);
    }

    public void setAllLedColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        this.updateLEDs();
    }

    // Returns the length of the LED Strip
    public int length() {
        return m_ledBuffer.getLength();
    }

    // Set all LEDs to OFF
    public void clearLEDs() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        this.updateLEDs();
    }

}
