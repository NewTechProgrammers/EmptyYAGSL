package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputConstants;

public class LEDSubsystem extends SubsystemBase {
    private final DigitalOutput redPWM;
    private final DigitalOutput greenPWM;
    private final DigitalOutput bluePWM;

    public LEDSubsystem()
    {
        redPWM = new DigitalOutput(DigitalInputConstants.kRedLedPort);
        greenPWM = new DigitalOutput(DigitalInputConstants.kGreenLedPort);
        bluePWM = new DigitalOutput(DigitalInputConstants.kBlueLedPort);
    }

    public void setColor(boolean r, boolean g, boolean b) {
        redPWM.set(!r);
        greenPWM.set(!g);
        bluePWM.set(!b);
    }

    public void turnOff() {
        setColor(false, false, false);
    }
    
}
