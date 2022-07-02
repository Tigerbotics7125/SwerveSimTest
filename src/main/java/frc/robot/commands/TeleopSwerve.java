/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import io.github.tigerbotics7125.tigerlib.util.CleanSupplier;

public class TeleopSwerve extends CommandBase {

    public enum RotationStyle {
        ROTATE,
        POINT
    }

    private Swerve s_Swerve;
    private boolean fieldRelative;
    private RotationStyle rotationStyle;
    private boolean openLoop;
    private CleanSupplier<Double> translationNorth;
    private CleanSupplier<Double> translationEast;
    private CleanSupplier<Double> rotationNorth;
    private CleanSupplier<Double> rotationEast;

    /**
     * @param s_Swerve Swerver subsystem
     * @param fieldRelative is robot field oriented?
     * @param openLoop is motor control open loop?
     * @param rotationStyle How to interperate rotation input:
     *     <p>ROTATE: east axis will give a percent output to rotation.
     *     <p>POINT: determine desired heading via atan2 and use PID to achieve it.
     *     <p>
     * @param translationNorth translate forwards and backwards, north positive.
     * @param translationEast translate leftwards and rightwards, east positive.
     * @param headingNorth heading point forwards and backwards, north positive.
     * @param headingEast heading point leftwards and rightwards, east positive.
     */
    public TeleopSwerve(
            Swerve s_Swerve,
            boolean fieldRelative,
            boolean openLoop,
            RotationStyle rotationStyle,
            CleanSupplier<Double> translationNorth,
            CleanSupplier<Double> translationEast,
            CleanSupplier<Double> headingNorth,
            CleanSupplier<Double> headingEast) {
        this.s_Swerve = s_Swerve;
        addRequirements(this.s_Swerve);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.rotationStyle = rotationStyle;
        this.translationNorth = translationNorth;
        this.translationEast = translationEast;
        this.rotationNorth = headingNorth;
        this.rotationEast = headingEast;
    }

    @Override
    public void execute() {
        // Robot x axis is forwards, so robot north is x axis.
        Translation2d translation =
                new Translation2d(-translationNorth.get(), translationEast.get())
                        .times(Constants.Swerve.maxSpeed);

        double rotation = 0;
        switch (rotationStyle) {
            case ROTATE:
                {
                    rotation = rotationEast.get() * Constants.Swerve.maxAngularVelocity;
                    break;
                }
            case POINT:
                {
                    double desiredAngle = Math.atan2(rotationEast.get(), rotationNorth.get());
                    // PID is backwards, idk why
                    rotation =
                            Constants.Swerve.headingPID.calculate(
                                    s_Swerve.getYaw().getRadians(), desiredAngle);
                    break;
                }
        }

        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
