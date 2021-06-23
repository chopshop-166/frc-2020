package frc.robot.logger;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import org.influxdb.InfluxDB;
import org.influxdb.InfluxDBFactory;
import org.influxdb.dto.Point;
import org.influxdb.dto.Query;

public class RobotLogger {
    private final InfluxDB influxDB;
    private final Map<String, SubsystemLogger> subsystems = new HashMap<>();

    // Store the robot name to tag all of our measurements with
    private final String robotName;

    public RobotLogger(final String robotName) {
        this.robotName = robotName;
        influxDB = InfluxDBFactory.connect("");

        final String databaseName = "FIRST";
        influxDB.query(new Query("CREATE DATABASE " + databaseName));
        influxDB.setDatabase(databaseName);
    }

    // Helper function that starts creating the point with boilerplate
    public Point.Builder createPoint(final String subSystem) {
        return Point.measurement("Robot").time(System.currentTimeMillis(), TimeUnit.MILLISECONDS).tag("Name", robotName)
                .tag("Subsystem", subSystem);
    }

    public void writePoint(final Point.Builder point) {
        // Build the point and write it into the database
        influxDB.write(point.build());
    }

    public SubsystemLogger addSubsystem(final String subsystemName) {
        final SubsystemLogger subsys = new SubsystemLogger(this, subsystemName);
        subsystems.put(subsystemName, subsys);
        return subsys;
    }

    // Accessor for SubsystemLogger by subsystem name
    public SubsystemLogger getSubsystem(final String subsystemName) {
        return subsystems.get(subsystemName);
    }

    public void updateFields() {
        for (final Map.Entry<String, SubsystemLogger> subsystem : subsystems.entrySet()) {
            subsystem.getValue().updateFields();
        }
    }
}