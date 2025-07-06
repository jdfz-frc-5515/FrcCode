// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.UpperSystem2025Cmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Candle2025.Candle2025;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Intake2025.Intake2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;
import frc.robot.commands.GoToCoralCmd;
import frc.robot.commands.fineTuneDrivetrainCmd;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public static final ImprovedCommandXboxController m_driverController = new ImprovedCommandXboxController(0);
    public static final ImprovedCommandXboxController m_driverController2 = new ImprovedCommandXboxController(1);
    public static final ImprovedCommandXboxController m_driverController3 = new ImprovedCommandXboxController(2);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final GroundIntakeSubsystem gIntakeSubsystem = new GroundIntakeSubsystem();

    TurningArm2025 m_turningArm = new TurningArm2025();
    Elevator2025 m_elevator = new Elevator2025();
    Intake2025 m_intake = new Intake2025();
    Candle2025 m_candle = new Candle2025();

    private List<Trigger> pathplannerEvents = new ArrayList<Trigger>();
    
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


        new UpperSystem2025Cmd(
            drivetrain,
            m_turningArm, m_elevator, m_intake, gIntakeSubsystem, m_candle
        );

        configureDriver1Bindings();
        configureDriver2Bindings();
        registerPathplannerEventsAndNamedCommands();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDriver1Bindings() {
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

        UpperSystem2025Cmd cmd = UpperSystem2025Cmd.inst;

        Trigger zeroUpperPosBtn = null;
        Trigger aimGroundCoralBtn = m_driverController2.a();
        Trigger aimLeftCoralBtn = m_driverController2.leftBumper();
        Trigger aimRightCoralBtn = m_driverController2.rightBumper();
        Trigger intakeBtn = m_driverController2.y();        // 启动intake,从上面漏斗intake
        Trigger groundIntakeSwitchBtn = m_driverController2.x(); // 启动/收起 地吸
        Trigger catchBallBtn = null;    // 抓球模式下，启动抓球功能
        Trigger toggleBallBtn = null;   // 切换抓球还是射珊瑚

        cmd.setResetToZeroPosTrigger(zeroUpperPosBtn);
        cmd.setAimLeftCoralTrigger(aimLeftCoralBtn);
        cmd.setAimRightCoralTrigger(aimRightCoralBtn);
        cmd.setIntakeTrigger(intakeBtn);
        cmd.setCatchBallTrigger(catchBallBtn);
        cmd.setToggleBallTrigger(toggleBallBtn);
        cmd.setGroundIntakeSwitchTrigger(groundIntakeSwitchBtn);

        aimGroundCoralBtn.whileTrue(new GoToCoralCmd(drivetrain));

        // m_driverController.a().onTrue(new InstantCommand(() -> {
        //     System.out.println("00000000000000000000000000000000000");
        //     gIntakeSubsystem.zeroCC();
        // }));

        // reset the field-centric heading on left bumper press
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        m_driverController.start().onTrue(new InstantCommand(() -> {
            drivetrain.resetHeadingForOdo(0);
        }));

        m_driverController.povUp().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 0));
        m_driverController.povLeft().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 1));
        m_driverController.povDown().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 2));
        m_driverController.povRight().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 3));
        // m_driverController.povUp() 
    }

    private void configureDriver2Bindings() {
        m_driverController2.a().onTrue(new InstantCommand(() -> {
            ControlPadHelper.setControlPadInfoData(-10, 1, -10);
        }));
        m_driverController2.x().onTrue(new InstantCommand(() -> {
            ControlPadHelper.setControlPadInfoData(-10, 2, -10);
        }));
        m_driverController2.y().onTrue(new InstantCommand(() -> {
            ControlPadHelper.setControlPadInfoData(-10, 3, -10);
        }));

        // m_driverController2.b().onTrue(new InstantCommand(() -> {
        //     ControlPadHelper.setControlPadInfoData(-10, 2, -10);
        // }));
    }

    private void configureDriver3Bindings() {
        var cmd = UpperSystem2025Cmd.inst;

        cmd.setArmTuningUpTrigger(m_driverController3.povRight());
        cmd.setArmTuningDownTrigger(m_driverController3.povLeft());

        cmd.setElevatorTuningDownTrigger(m_driverController3.povDown());
        cmd.setElevatorTuningUpTrigger(m_driverController3.povUp());

        cmd.setResetCanCodePositionTrigger(m_driverController3.start());
        cmd.setLockElevatorTrigger(m_driverController3.b());
        cmd.setUnlockElevatorTrigger(m_driverController3.a());
    }

    private void registerPathplannerEventsAndNamedCommands() {
        pathplannerEvents.add(new EventTrigger("LN").onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateLn();
        })));
        pathplannerEvents.add(new EventTrigger(("Raise2L1")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL1();
        })));
        pathplannerEvents.add(new EventTrigger(("Raise2L2")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL2();
        })));
        pathplannerEvents.add(new EventTrigger(("Raise2L3")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL3();
        })));
        pathplannerEvents.add(new EventTrigger(("Raise2L4")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL4();
        })));

        pathplannerEvents.add(new EventTrigger(("Ball1")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateBall1();
        })));

        pathplannerEvents.add(new EventTrigger(("Ball2")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateBall2();
        })));

        pathplannerEvents.add(new EventTrigger(("Intake")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.startIntake();
        })));
        pathplannerEvents.add(new EventTrigger(("Shoot")).onTrue(new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.startShoot();
        })));

        NamedCommands.registerCommand("LN", new InstantCommand(()->{
            UpperSystem2025Cmd.inst.setStateLn();
        }));
        NamedCommands.registerCommand("Raise2L1",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL1();
        }));
        NamedCommands.registerCommand("Raise2L2",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL2();
        }));
        NamedCommands.registerCommand("Raise2L3",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL3();
        }));
        NamedCommands.registerCommand("Raise2L4",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateL4();
        }));

        NamedCommands.registerCommand("Ball1",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateBall1();
        }));

        NamedCommands.registerCommand("Ball2",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.setStateBall2();
        }));

        NamedCommands.registerCommand("Intake",new InstantCommand(() -> {
            UpperSystem2025Cmd.inst.startIntake();
        }));
        // NamedCommands.registerCommand("Shoot",new InstantCommand(() -> {
        //     UpperSystem2025Cmd.inst.startShoot();
        // }));

        
        String[] list = GlobalConfig.getAllAimPathNames();
        for (int i = 0; i < list.length; ++i) {
            String name = list[i];
            // pathplannerEvents.add(new EventTrigger((name)).onTrue(new MoveToByPath2025Cmd(m_moveToSubSys, s_Swerve, name)));
            NamedCommands.registerCommand(name, 
                new SequentialCommandGroup(
                    // new InstantCommand(() -> {
                    //     if (name.startsWith("ap")) {
                    //         GlobalConfig.ExtractApPathName(name);
                    //     }
                    // }), 
                    // new MoveToByPath2025Cmd(m_moveToSubSys, s_Swerve, name),
                    // new WaitShooterCmd2025(m_elevator),
                    // new InstantCommand(() -> {
                    //     // System.out.println("auto shoot ----------------------->>>>>>>>>>>>>>>");
                    //     UpperSystem2025Cmd.inst.startShoot();
                    // })
                )
            );
            
            // NamedCommands.registerCommand(name,new InstantCommand(()->{}));
        }

        // NamedCommands.registerCommand("SetHead", new InstantCommand(()->{
            
        // }));
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


    public void telInit() {
        LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 0);
        LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 0);
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 0);

        UpperSystem2025Cmd.inst.schedule();
    }

    public void autoInit() {
        UpperSystem2025Cmd.inst.schedule();

        // UpperSystem2025Cmd.inst.getRequirements().forEach(sys -> {
        //     System.out.println("up: " + sys.getName());
        // });
    }

    public void testInit() {
        configureDriver3Bindings();
        UpperSystem2025Cmd.inst.schedule();
    }

    public void onDisabled() {

        LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 2);
        LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 2);
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 2);
    }
}
