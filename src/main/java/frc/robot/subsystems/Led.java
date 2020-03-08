package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.ISolenoid;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.LEDMap;

public class Led extends SubsystemBase {
    AddressableLED liftLED;
    ISolenoid ringLight;
    AddressableLEDBuffer liftBuffer;
    double firstHue;

    public Led(LEDMap map) {

        liftBuffer = new AddressableLEDBuffer(map.liftBufferLength());
        liftLED = map.liftLED();
        ringLight = map.visionRingLight();

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

    public CommandBase visionLedStream() {
        CommandBase cmd = new InstantCommand(() -> {
            for (var i = 0; i < liftBuffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (0 + (i * 180 / liftBuffer.getLength())) % 180;
                // Set the value
                liftBuffer.setHSV(i, hue, 255, 255);
            }
            // Increase by to make the rainbow "move"
            firstHue += 3;
            // Check bounds
            firstHue %= 180;

        }, this);
        cmd.setName("Vision Stream");
        return cmd;
    }
}