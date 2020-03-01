package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.LEDMap;

public class Led extends SubsystemBase {
    AddressableLED visionLED;
    AddressableLEDBuffer visionBuffer = new AddressableLEDBuffer(6);

    public Led(LEDMap map) {
        visionLED = map.visionLED();
    }

    public CommandBase visionGreenOn() {
        CommandBase cmd = new InstantCommand(() -> {
            for (var i = 0; i < visionBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for green
                visionBuffer.setRGB(i, 0, 255, 0);
            }

            visionLED.setData(visionBuffer);
            visionLED.start();
        }, this);
        cmd.setName("Vision Green");
        return cmd;
    }

    public CommandBase ledOff() {
        CommandBase cmd = new InstantCommand(() -> {
            visionLED.stop();
        }, this);
        cmd.setName("LED Off");
        return cmd;
    }
}