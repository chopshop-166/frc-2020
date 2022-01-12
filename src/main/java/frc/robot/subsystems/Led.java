package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.pneumatics.ISolenoid;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.maps.RobotMap.LEDMap;

public class Led extends SmartSubsystemBase {
    AddressableLED robotLED;
    ISolenoid ringLight;
    AddressableLEDBuffer robotLedBuffer;

    public Led(LEDMap map) {

        robotLedBuffer = new AddressableLEDBuffer(map.RobotBufferLength());
        robotLED = map.robotLED();
        ringLight = map.visionRingLightSolenoid();

    }

    public CommandBase ringLightOn() {
        CommandBase cmd = new InstantCommand(() -> {
            ringLight.set(true);
        }, this);
        cmd.setName("Ring Light On");
        return cmd;
    }

    public CommandBase ringLightOff() {
        CommandBase cmd = new InstantCommand(() -> {
            ringLight.set(false);
        }, this);
        cmd.setName("Ring Light Off");
        return cmd;
    }

    public CommandBase robotLedColor(int hue, int saturation, int value) {
        CommandBase cmd = new InstantCommand(() -> {
            for (var i = 0; i < robotLedBuffer.getLength(); i++) {

                robotLedBuffer.setHSV(i, hue, saturation, value);
            }
        }, this);
        cmd.setName("Vision Stream");
        return cmd;
    }

    @Override
    public void safeState() {
        // None
    }
}