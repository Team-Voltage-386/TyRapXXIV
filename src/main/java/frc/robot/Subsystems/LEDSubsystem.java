package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.DisabledLEDCommand;
import frc.robot.Constants.LEDs;

public class LEDSubsystem extends SubsystemBase {

    // Addresable LED
    private AddressableLED m_led = new AddressableLED(LEDs.kpwm);

    // The LED Buffer
    // Reuse buffer
    // start empty output
    // Length is expensive to set, so only set it once, then just update data
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDs.kLEDBufferLength);

    private DisabledLEDCommand m_DisabledLEDCommand;

    public LEDSubsystem() {
        m_DisabledLEDCommand = new DisabledLEDCommand(this);
    }

    public DisabledLEDCommand getDisabledLEDCommand() {
        return this.m_DisabledLEDCommand;
    }

    public void setLedColor(int index, int r, int g, int b) {
        m_ledBuffer.setRGB(index, r, g, b);
    }

    public void setAllLedColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
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
    }

}
