package frc.robot.logger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.influxdb.dto.Point;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

class InfluxSendable implements SendableBuilder {

    private final Map<String, String> tagMap;
    private final Map<String, BooleanSupplier> booleanSupplierMap = new HashMap<>();
    private final Map<String, DoubleSupplier> doubleSupplierMap = new HashMap<>();
    private final Map<String, Supplier<String>> stringSupplierMap = new HashMap<>();

    public InfluxSendable(final Map<String, String> tags) {
        tagMap = new HashMap<>(tags);
    }

    @Override
    public void setSmartDashboardType(final String type) {
        tagMap.put("Type", type);
    }

    @Override
    public void setActuator(final boolean value) {
        // Left Empty On Purpose
    }

    @Override
    public void setSafeState(final Runnable func) {
        // Left Empty On Purpose
    }

    @Override
    public void setUpdateTable(final Runnable func) {
        // Left Empty On Purpose
    }

    @Override
    public NetworkTableEntry getEntry(final String key) {
        // Left Empty On Purpose
        return null;
    }

    @Override
    public void addBooleanProperty(final String key, final BooleanSupplier getter, final BooleanConsumer setter) {
        if (getter != null) {
            booleanSupplierMap.put(key, getter);
        }
    }

    @Override
    public void addDoubleProperty(final String key, final DoubleSupplier getter, final DoubleConsumer setter) {
        if (getter != null) {
            doubleSupplierMap.put(key, getter);
        }
    }

    @Override
    public void addStringProperty(final String key, final Supplier<String> getter, final Consumer<String> setter) {
        if (getter != null) {
            stringSupplierMap.put(key, getter);
        }
    }

    @Override
    public void addBooleanArrayProperty(final String key, final Supplier<boolean[]> getter,
            final Consumer<boolean[]> setter) {
        // Left Empty On Purpose
    }

    @Override
    public void addDoubleArrayProperty(final String key, final Supplier<double[]> getter,
            final Consumer<double[]> setter) {
        // Left Empty On Purpose
    }

    @Override
    public void addStringArrayProperty(final String key, final Supplier<String[]> getter,
            final Consumer<String[]> setter) {
        // Left Empty On Purpose
    }

    @Override
    public void addRawProperty(final String key, final Supplier<byte[]> getter, final Consumer<byte[]> setter) {
        // Left Empty On Purpose
    }

    @Override
    public void addValueProperty(final String key, final Supplier<NetworkTableValue> getter,
            final Consumer<NetworkTableValue> setter) {
        // Left Empty On Purpose
    }

    // Iterate over keyMap and add all of the keys
    // Iterate over the valueSupplierMap and get all of the values in the correct
    // form
    // This should probably accept a point that's already been created for us.
    public void updateValues(final Point.Builder point) {
        // Add tags
        point.tag(tagMap);
        // Get any booleans that this Sendable provides
        for (final Map.Entry<String, BooleanSupplier> entry : booleanSupplierMap.entrySet()) {
            point.addField(entry.getKey(), entry.getValue().getAsBoolean());
        }
        // Get any doubles that this Sendable provides
        for (final Map.Entry<String, DoubleSupplier> entry : doubleSupplierMap.entrySet()) {
            point.addField(entry.getKey(), entry.getValue().getAsDouble());
        }
        // Get any strings that this Sendable provides
        for (final Map.Entry<String, Supplier<String>> entry : stringSupplierMap.entrySet()) {
            point.addField(entry.getKey(), entry.getValue().get());
        }
    }

    @Override
    public NetworkTable getTable() {
        return null;
    }
}