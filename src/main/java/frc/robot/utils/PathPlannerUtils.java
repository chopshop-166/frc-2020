package frc.robot.utils;

import java.util.Arrays;
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
    public static Trajectory.State processLine(String line) {
        Trajectory.State state = new Trajectory.State();
        double[] lineSplit = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();

        if (lineSplit.length != 7) {
            return state;
        }
        state.timeSeconds = lineSplit[0];
        state.velocityMetersPerSecond = lineSplit[1];
        state.accelerationMetersPerSecondSq = lineSplit[2];

        // Fill in pose information
        double x = 0, y = 0, rotation = 0;
        x = lineSplit[3];
        y = lineSplit[4];
        rotation = Math.toRadians(lineSplit[5]);

        Pose2d pose = new Pose2d(x, y, new Rotation2d(rotation));
        state.poseMeters = pose;
        state.curvatureRadPerMeter = 1 / lineSplit[6];
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
