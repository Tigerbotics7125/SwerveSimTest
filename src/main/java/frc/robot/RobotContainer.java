/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.JoystickSupplier;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopSwerve.RotationStyle;
import frc.robot.subsystems.swerve.Swerve;
import tigerlib.input.controller.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final XboxController driver = new XboxController(0);

    // Subsystems
    Swerve s_Swerve = new Swerve();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        boolean fieldRelative = true;
        boolean openLoop = true;
        RotationStyle rotationStyle = RotationStyle.ROTATE;
        // left joystick is translation, right joystick is rotation
        JoystickSupplier translationNorth = () -> driver.leftY().getVal();
        JoystickSupplier translationEast = () -> driver.leftX().getVal();
        JoystickSupplier rotationNorth = () -> driver.rightY().getVal();
        JoystickSupplier rotationEast = () -> driver.rightX().getVal();
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        fieldRelative,
                        openLoop,
                        rotationStyle,
                        translationNorth,
                        translationEast,
                        rotationNorth,
                        rotationEast));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
