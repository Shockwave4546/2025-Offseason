package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    private final Servo m_ExtensionWingServo;

    public ServoSubsystem(int kServoPWMPort) {
        // uses the PWM port defined in the Constants file
        m_ExtensionWingServo = new Servo(kServoPWMPort);
    }

    // Single operation mode to only set the position for now.  We can setup a set Angle down the road if needed (not for this but for more generic purposes)
    public void setServoPosition(double position) {
        m_ExtensionWingServo.set(position);
    }
}
