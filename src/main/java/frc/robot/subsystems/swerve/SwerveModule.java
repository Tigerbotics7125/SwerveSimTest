/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.angleGearRatio;
import static frc.robot.Constants.Swerve.angleMotorInvert;
import static frc.robot.Constants.Swerve.angleNeutralMode;
import static frc.robot.Constants.Swerve.driveGearRatio;
import static frc.robot.Constants.Swerve.driveMotorInvert;
import static frc.robot.Constants.Swerve.driveNeutralMode;
import static frc.robot.Constants.Swerve.maxSpeed;
import static frc.robot.Constants.Swerve.wheelCircumference;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {
    public int moduleNumber;

    private double angleOffset;
    private WPI_TalonFX angleMotor;
    private WPI_TalonFX driveMotor;
    private WPI_CANCoder angleEncoder;
    private double lastAngle;

    // Simulation only
    private TalonFXSimCollection sim_angleMotorSim;
    private TalonFXSimCollection sim_driveMotorSim;
    private FlywheelSim sim_angleModModel;
    private FlywheelSim sim_driveModModel;

    // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        if (Robot.isSimulation()) {
            simulationInit();
        }

        lastAngle = getState().angle.getDegrees();
    }

    private void simulationInit() {
        sim_angleMotorSim = angleMotor.getSimCollection();
        sim_driveMotorSim = driveMotor.getSimCollection();

        sim_angleModModel = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(0.30, 0.1),
                DCMotor.getFalcon500(1),
                angleGearRatio);
        sim_driveModModel = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(.1, .3),
                DCMotor.getFalcon500(1),
                driveGearRatio);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // don't move the module greater than 90 degrees, as it is unnecisarey
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        /* Drive */
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
            driveMotor.set(
                    ControlMode.Velocity,
                    velocity);
                    /*
                    DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
                    */
                }

        /* Angle */
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees();
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, angleGearRatio));
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                getCANCoderAbsRotation().getDegrees() - angleOffset, angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);

        angleEncoder.setStatusFramePeriod(
                CANCoderStatusFrame.SensorData, Constants.CAN.ctreCANCoderStatus1Slow);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(angleMotorInvert);
        angleMotor.setNeutralMode(angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(driveMotorInvert);
        driveMotor.setNeutralMode(driveNeutralMode);
    }

    public Rotation2d getCANCoderAbsRotation() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetDriveDistance() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(
                        angleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    public double getVelocityMPS() {
        return Conversions.falconToMPS(
                driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMPS(), getRotation());
    }

    public Pose2d getPose(Pose2d swerve, Translation2d translation) {
        return swerve.plus(new Transform2d(translation, getRotation()));
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        sim_angleMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        sim_driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

        sim_angleModModel.setInputVoltage(
                sim_angleMotorSim.getMotorOutputLeadVoltage());
        sim_driveModModel.setInputVoltage(
                sim_driveMotorSim.getMotorOutputLeadVoltage());

        sim_angleModModel.update(0.02);
        sim_driveModModel.update(0.02);

        sim_angleMotorSim.addIntegratedSensorPosition((int) Conversions.RPMToFalcon(sim_angleModModel.getAngularVelocityRPM() * .02, angleGearRatio));
        sim_driveMotorSim.addIntegratedSensorPosition((int) Conversions.RPMToFalcon(sim_driveModModel.getAngularVelocityRPM() * .02, driveGearRatio));
        sim_angleMotorSim.setIntegratedSensorVelocity((int) Conversions.RPMToFalcon(sim_angleModModel.getAngularVelocityRPM(), angleGearRatio));
        sim_driveMotorSim.setIntegratedSensorVelocity((int) Conversions.RPMToFalcon(sim_driveModModel.getAngularVelocityRPM(), driveGearRatio));

        System.out.println("mod " + moduleNumber + ": - " + Timer.getFPGATimestamp());
        System.out.println(getState().toString());
        System.out.println("angle input Volts: " + sim_angleMotorSim.getMotorOutputLeadVoltage());
        System.out.println("drive input Volts: " + sim_driveMotorSim.getMotorOutputLeadVoltage());
        System.out.println("angle model RPM: " + sim_angleModModel.getAngularVelocityRPM());
        System.out.println("drive model RPM: " + sim_driveModModel.getAngularVelocityRPM());
        System.out.println();
    }
}
