/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.lib.math;

public class Conversions {

    /**
     * @param counts Falcon encoder counts
     * @param gearRation Gear ratio between falcon and mechanism
     * @return rotation of mechanism in degrees
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Rotation of mechanism in degrees
     * @param gearRatio Gear ratio between falcon and mechanism
     * @return Falcon encoder counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon velocity counts
     * @param gearRatio Gear ratio between falcon and mechanism
     * @return RPM of mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear ratio between falcon and mechanism
     * @return Falcon velocity counts
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon velocity counts
     * @param circumference Circumference of wheel
     * @param gearRatio Gear ratio between falcon and mechanism
     * @return Linear speed of mechanism
     */
    public static double falconToMPS(
            double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of wheel
     * @param gearRatio Gear ratio between falcon and mechanism
     * @return Falcon velocity counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
}
