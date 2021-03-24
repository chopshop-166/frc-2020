package frc.robot.utils;

import java.util.Collections;
import java.util.Scanner;
import java.util.Vector;
import java.io.BufferedReader;
import java.io.IOException;
import java.lang.Math;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class PathPlannerUtils {

    private final static String COMMA_DELIMITER = ",";

    public static Trajectory.State processLine(String line) {
        Trajectory.State state = new Trajectory.State();
        try (Scanner lineScanner = new Scanner(line)) {
            lineScanner.useDelimiter(COMMA_DELIMITER);
            if (lineScanner.hasNext()) {
                state.timeSeconds = lineScanner.nextDouble();
            }
            if (lineScanner.hasNext()) {
                state.velocityMetersPerSecond = lineScanner.nextDouble();
            }
            if (lineScanner.hasNext()) {
                state.accelerationMetersPerSecondSq = lineScanner.nextDouble();
            }

            // Fill in pose information
            double x = 0, y = 0, rotation = 0;
            if (lineScanner.hasNext()) {
                x = lineScanner.nextDouble();
            }
            if (lineScanner.hasNext()) {
                y = lineScanner.nextDouble();
            }
            if (lineScanner.hasNext()) {
                rotation = Math.toRadians(lineScanner.nextDouble());
            }

            Pose2d pose = new Pose2d(x, y, new Rotation2d(rotation));
            state.poseMeters = pose;

            if (lineScanner.hasNext()) {
                state.curvatureRadPerMeter = 1 / lineScanner.nextDouble();
            }
        }
        return state;
    }

    public static Trajectory fromPathPlannerCSV(Path path) throws IOException {
        try (BufferedReader reader = Files.newBufferedReader(path)) {
            Vector<Trajectory.State> stateList = new Vector<Trajectory.State>();
            String line;
            while ((line = reader.readLine()) != null) {
                stateList.add(processLine(line));
            }
            return new Trajectory(Collections.list((stateList.elements())));
        }
    }
}
