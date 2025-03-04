package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class AutoposHelpers {
    private static final String CONFIG_PATH = "/home/lvuser/deploy/autopos/coordinates.json";
    private static AutoposConfig autoposConfig;

    public static void loadConfig() {
        try {
            ObjectMapper objectMapper = new ObjectMapper();
            autoposConfig = objectMapper.readValue(new File(CONFIG_PATH), AutoposConfig.class);
            System.out.println("AUTOPOS: CONFIG LOADED SUCCESSFULLY!");
        } catch (IOException e) {
            System.err.println("AUTOPOS: FAILED TO LOAD AUTOPOS CONFIG: " + e.getMessage());
            autoposConfig = new AutoposConfig();
        }
    }

    public static AutoposConfig getConfig() {
        return autoposConfig;
    }

    /**
     * Retrieves the full AutoposSidePosition for a given ID
     * 
     * @param alliance "red" or "blue" to specify the alliance
     * @param id       The key inside the alliance (e.g., "19", "18").
     * @param subKey   Subkey ("left", "right")
     * @return Position object or a default (0,0) position if not found.
     */
    public static AutoposSidePosition getSidePosition(String alliance, String id) {
        if (autoposConfig == null) {
            System.err.println("AUTOPOSHELPERS: CONFIG NOT LOADED!");
            return new AutoposSidePosition();
        }

        try {
            if (alliance == "red") {
                Map<String, AutoposSidePosition> redAlliance = autoposConfig.red_alliance;
                if (redAlliance.containsKey(id)) {
                    return redAlliance.getOrDefault(id, new AutoposSidePosition());
                }
            } else if (alliance == "blue") {
                Map<String, AutoposSidePosition> blueAlliance = autoposConfig.blue_alliance;
                if (blueAlliance.containsKey(id)) {
                    return blueAlliance.getOrDefault(id, new AutoposSidePosition());
                }
            }
        } catch (Exception e) {
            System.err.println("AUTOPOSHELPERS: ERROR RETRIEVING POSITION: " + e.getMessage());
        }

        return new AutoposSidePosition();
    }

    /**
     * Retrieves the Position object from an AutoposSidePosition object based on the side
     * 
     * @param sidePosition The AutoposSidePosition object.
     * @param side         "left" or "right"
     * @return Position object or a default (0,0) position if not found.
     */
    public static Position getPositionFromSidePosition(AutoposSidePosition sidePosition, String side) {
        return (sidePosition == null) ? new Position()
                : side == "left" ? sidePosition.left : side == "right" ? sidePosition.right : new Position();
    }

    /**
     * Retrieves the heading from an AutoposSidePosition object
     * 
     * @param sidePosition The AutoposSidePosition object.
     * @return The heading value or a default (0,0) position if not found.
     */
    public static double getHeading(AutoposSidePosition sidePosition) {
        return (sidePosition != null) ? sidePosition.heading : 0.0;
    }

    /**
     * Generates a Pose2d object based on the given AutoPosSidePosition object and selected side
     * 
     * @param sidePosition The AutoposSidePosition object.
     * @param side         "left" or "right"

     * @return The heading value or a default (0,0) position if not found.
     */
    public static Pose2d getPose2d(AutoposSidePosition sidePosition, String side) {
        if (sidePosition == null) {
            return new Pose2d();
        }

        Vector<N2> positionVector = "left".equals(side) ? sidePosition.left.getVector() :
                                    "right".equals(side) ? sidePosition.right.getVector() : null;

        if (positionVector == null) {
            return new Pose2d(new Translation2d(), Rotation2d.fromDegrees(sidePosition.heading));
        }

        return new Pose2d(new Translation2d(positionVector), Rotation2d.fromDegrees(sidePosition.heading));
    }


    public static class AutoposConfig {
        public Map<String, AutoposSidePosition> red_alliance;
        public Map<String, AutoposSidePosition> blue_alliance;
    }

    public static class AutoposSidePosition {
        public Position left = new Position();
        public Position right = new Position();
        public double heading = 0.0;
    }

    public static class Position {
        public double x = 0.0, y = 0.0;

        public Vector<N2> getVector() {
            return VecBuilder.fill(this.x, this.y);
        }
    }

}
