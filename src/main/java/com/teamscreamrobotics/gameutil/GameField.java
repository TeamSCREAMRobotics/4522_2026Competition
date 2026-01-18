package com.teamscreamrobotics.gameutil;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameField extends SubsystemBase {
    public static enum Zones {
        LEFTSCORE(0, 4.0345, 4.0651582908, 8.069),
        RIGHTSCORE(0, 0, 4.0651582908, 4.0345),
        LEFTNEUTRAL(5.22258925, 4.0345, 11.31858925, 8.069),
        RIGHTNEUTRAL(5.22258925, 0, 11.31858925, 4.0345),
        LEFTOTHER(12.479313243, 4.0345, 16.5411785, 8.069),
        RIGHTOTHER(12.479313243, 0, 16.5411785, 4.0345),
        OURTRENCH(4.0651582908, 0, 5.1895217266, 8.069),
        THEIRTRENCH(11.3516779062, 0, 12.4760202092, 8.069);

        /*
         * The i before the variable stands for initial
         * The f before the variable stands for final
         * 
         * x stands for length
         * y stands for width
         */

        private final double ix;
        private final double iy;
        private final double fx;
        private final double fy;

        Zones(double ix, double iy, double fx, double fy) {
            this.ix = ix;
            this.iy = iy;
            this.fx = fx;
            this.fy = fy;
        }
    }

    public static Zones getZones(Supplier<Pose2d> robotPose) {
        double robotX = robotPose.get().getX();
        double robotY = robotPose.get().getY();
        
        boolean isBlueAlliance = DriverStation.getAlliance()
            .filter(alliance -> alliance == Alliance.Blue)
            .isPresent();
        
        // Full field length in meters (max X of your zones)
        final double FIELD_LENGTH = 16.5411785;
        
        // Flip X for red alliance (mirror across field centerline)
        if (!isBlueAlliance) {
            robotX = FIELD_LENGTH - robotX;
        }
    
        // Check which zone the robot is inside
        for (Zones zone : Zones.values()) {
            if (robotX >= zone.ix && robotX <= zone.fx &&
                robotY >= zone.iy && robotY <= zone.fy) {
                return zone;
            }
        }
    
        // Robot is not in any defined zone
        return null;
    }

}
