// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.MoveToCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.commands.GoToCoralCmd;
import frc.robot.commands.fineTuneDrivetrainCmd;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public static final ImprovedCommandXboxController m_driverController = new ImprovedCommandXboxController(0);
    // public static final XboxController traditionalController = new XboxController(0);
    // private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher;
    private final StructPublisher<Pose2d> robotPospublisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

    public RobotContainer() {
        ControlPadHelper.init();

        swerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/MyStates", SwerveModuleState.struct).publish();

        drivetrain.setDefaultCommand(
            drivetrain.run(() -> drivetrain.driveFieldCentric(m_driverController))
        );

        configureBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driverController.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driverController.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // m_driverController.x().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // m_driverController.y().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driverController.rightBumper().onTrue(new InstantCommand(() -> {
        //     logger.flush();
        // }));

        
        m_driverController.a().whileTrue(new MoveToCmd(drivetrain, m_driverController));
        m_driverController.b().whileTrue(new GoToCoralCmd(drivetrain));
        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // m_driverController.rightBumper().whileTrue(new FaceObjectCmd(drivetrain));
        m_driverController.rightBumper().onTrue(new InstantCommand(() -> {
            drivetrain.resetHeadingForOdo(0);
        }));
        m_driverController.povUp().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 0));
        m_driverController.povLeft().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 1));
        m_driverController.povDown().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 2));
        m_driverController.povRight().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 3));
        // m_driverController.povUp()
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

        public void update() {
        Pose2d pos = drivetrain.getPose();
        ControlPadHelper.publishRobotPos(pos);

        swerveStatePublisher.set(drivetrain.getModuleStates());
        robotPospublisher.set(pos);
    }

    public void updateAlways() {
        ControlPadHelper.update();
    }
}
