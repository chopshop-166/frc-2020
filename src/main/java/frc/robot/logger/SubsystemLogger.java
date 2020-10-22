package frc.robot.logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.influxdb.dto.Point;

import edu.wpi.first.wpilibj.Sendable;

public class SubsystemLogger {
    private final RobotLogger logger;
    private final String subSystemName;
    private final List<InfluxSendable> sendableList = new ArrayList<>();

    public SubsystemLogger(final RobotLogger logger, final String name) {
        this.logger = logger;
        this.subSystemName = name;
    }

    public Point.Builder createPoint() {
        return logger.createPoint(subSystemName);
    }

    public void writePoint(final Point.Builder point) {
        logger.writePoint(point);
    }

    // Register a sendable under this
    public void register(final Sendable sendable, final Map<String, String> tags) {
        final Map<String, String> subsysTags = new HashMap<>();
        subsysTags.putAll(tags);
        subsysTags.put("Subsystem", this.subSystemName);
        final InfluxSendable record = new InfluxSendable(subsysTags);
        sendable.initSendable(record);
        sendableList.add(record);
    }

    // Register a sendable under this
    public void register(final Sendable sendable) {
        final Map<String, String> subsysTags = new HashMap<>();
        subsysTags.put("Subsystem", this.subSystemName);
        final InfluxSendable record = new InfluxSendable(subsysTags);
        sendable.initSendable(record);
        sendableList.add(record);
    }

    public void updateFields() {
        for (final InfluxSendable sendable : sendableList) {
            final Point.Builder point = createPoint();
            sendable.updateValues(point);
            writePoint(point);
        }
    }

}