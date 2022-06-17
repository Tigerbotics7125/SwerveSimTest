/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase {

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModule[] swerveMods;
    private WPI_PigeonIMU gyro;

    private BasePigeonSimCollection sim_gyro;

    public Swerve() {
        gyro = new WPI_PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        poseEstimator =
                new SwerveDrivePoseEstimator(
                        getYaw(),
                        new Pose2d(),
                        Constants.Swerve.swerveKinematics,
                        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(.001, .001, .001),
                        new MatBuilder<>(Nat.N1(), Nat.N1()).fill(.001),
                        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(.001, .001, .001),
                        0.02);

        swerveMods =
                new SwerveModule[] {
                    new SwerveModule(0, Constants.Swerve.Mod0.constants),
                    new SwerveModule(1, Constants.Swerve.Mod1.constants),
                    new SwerveModule(2, Constants.Swerve.Mod2.constants),
                    new SwerveModule(3, Constants.Swerve.Mod3.constants)
                };

        if (Robot.isSimulation()) {
            sim_gyro = gyro.getSimCollection();
        }
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        var desiredModStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), translation.getY(), rotation, getYaw())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));

        // set module states
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredModStates[mod.moduleNumber], isOpenLoop);
        }
    }


    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d[] getModulePoses() {
        var poses = new Pose2d[swerveMods.length];

        double wheelBase = Constants.Swerve.wheelBase;
        double trackWidth = Constants.Swerve.trackWidth;
        var translations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        for (SwerveModule mod : swerveMods) {
            poses[mod.moduleNumber] = mod.getPose(this.getPose(), translations[mod.moduleNumber]);
        }

        return poses;
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose, getYaw());

        for (SwerveModule mod : swerveMods) {
            mod.resetDriveDistance();
        }
    }

    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[swerveMods.length];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - ypr[0])
                : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModuleStates());

        Robot.field.setRobotPose(getPose());
        Robot.field.getObject("Mods").setPoses(getModulePoses());

        for (SwerveModule mod : swerveMods) {

            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANCoder",
            // mod.getCANCoderAbsRotation().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    @Override
    public void simulationPeriodic() {
        var moduleStates = getModuleStates();

        var chassisSpeed = Constants.Swerve.swerveKinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        sim_gyro.addHeading(chassisRotationSpeed * .02);
    }
}
