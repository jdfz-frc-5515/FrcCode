// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.UpperSystem2025Cmd;
import frc.robot.commands.WaitForShootOverCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.PlayCandleSubsystem;
import frc.robot.subsystems.Candle2025.Candle2025;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Intake2025.Intake2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;
import frc.robot.utils.SmartDashboardEx;
import frc.robot.commands.GoToGroundCoralCmd;
import frc.robot.commands.fineTuneDrivetrainCmd;
import frc.robot.Constants.Candle;
import frc.robot.commands.CandleCmd;
import frc.robot.commands.FaceGroundCoralCmd;
import frc.robot.commands.FakeGoToCoralAndElevatorToLnCmd;
import frc.robot.commands.GoForward;


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
    // PlayCandleSubsystem m_candle = new PlayCandleSubsystem();
    Candle2025 m_candle = new Candle2025();

    private List<Trigger> pathplannerEvents = new ArrayList<Trigger>();
    
    private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher;
    private final StructPublisher<Pose2d> robotPospublisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

    private final double HEADING_RED = 0;
    private final double HEADING_BLUE = 180;
    Command m_autoPath;

    public RobotContainer() {
        FollowPathCommand.warmupCommand().schedule();
        ControlPadHelper.init();

        swerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/MyStates", SwerveModuleState.struct).publish();

        drivetrain.setDefaultCommand(
            drivetrain.run(() -> drivetrain.driveFieldCentric(m_driverController))
        );


        // setHeading here for auto 
        double headingAngle = HEADING_RED;  // HEADING_RED or HEADING_BLU=]0.;
        // // var alliance = DriverStation.getAlliance();
        // // if (alliance.isPresent()) {
        // //     switch (alliance.get()) {
        // //         case Blue:
        // //         headingAngle = 180;
        // //         break;
        // //         case Red:
        // //         headingAngle = 0;
        // //         break;
        // //     }
        // // }
        drivetrain.resetHeadingForOdo(headingAngle);

        new UpperSystem2025Cmd(
            drivetrain,
            m_candle,
            m_turningArm, m_elevator, m_intake, gIntakeSubsystem
        );

        configureDriver1Bindings();
        configureDriver2Bindings();
        registerPathplannerEventsAndNamedCommands();

        drivetrain.registerTelemetry(logger::telemeterize);
        
        m_autoPath = new PathPlannerAuto("RedMid");
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

        // debug
        // cmd.setGISensorTrigger(m_driverController.y());

        
        Trigger zeroUpperPosBtn = m_driverController.rightStick();
        Trigger aimGroundCoralBtn = m_driverController.a();
        Trigger aimLeftCoralBtn = m_driverController.leftBumper();
        Trigger aimRightCoralBtn = m_driverController.rightBumper();
        Trigger intakeBtn = m_driverController.x();        // 启动intake,从上面漏斗intake
        Trigger groundIntakeSwitchBtn = m_driverController.y(); // 启动/收起 地吸
        // Trigger catchBallBtn = m_driverController.b();    // 抓球模式下，启动抓球功能
        cmd.setResetToZeroPosTrigger(zeroUpperPosBtn);
        cmd.setAimLeftCoralTrigger(aimLeftCoralBtn);
        cmd.setAimRightCoralTrigger(aimRightCoralBtn);
        cmd.setIntakeTrigger(intakeBtn);    // 开启intake，shoot，Aglea模式下是挑球

        cmd.setSwitchCoralAndBallTrigger(m_driverController.b());

        cmd.setGroundIntakeSwitchTrigger(groundIntakeSwitchBtn);

        aimGroundCoralBtn.whileTrue(new GoToGroundCoralCmd(drivetrain, ()->{
            return cmd.getIsGroundIntakeCoralIn();
        }));

        // m_driverController.a().onTrue(new InstantCommand(() -> {
        //     System.out.println("00000000000000000000000000000000000");
        //     gIntakeSubsystem.zeroCC();
        // }));

        // reset the field-centric heading on left bumper press
        // m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.setFieldCentric()));
        

        m_driverController.start().onTrue(new InstantCommand(() -> {
            drivetrain.resetHeadingForOdo(0);
        }));

        m_driverController.povUp().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 0));
        m_driverController.povLeft().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 1));
        m_driverController.povDown().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 2));
        m_driverController.povRight().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 3));

        m_driverController.leftTrigger().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 4));
        m_driverController.rightTrigger().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 5));
        // m_driverController.povUp() 
    }

    private void configureDriver2Bindings() {
        UpperSystem2025Cmd cmd = UpperSystem2025Cmd.inst;
        cmd.setLnTrigger(m_driverController2.a(), m_driverController2.b(), m_driverController2.x(), m_driverController2.y(), m_driverController2.leftBumper());
        cmd.setSwitchIntakeSourceTrigger(m_driverController2.rightBumper());
        
        // cmd.setGroundIntakeOutTakeTrigger(m_driverController2.rightTrigger());

        cmd.setElevatorLevelDownTrigger(m_driverController2.povDown());
        cmd.setElevatorLevelUpTrigger(m_driverController2.povUp());

        cmd.setGroundIntakeOutTakeTrigger(m_driverController2.povLeft());

        m_driverController2.povRight().onTrue(new InstantCommand(() -> {
            cmd.toggleSpecialLed();
        }));
        // m_driverController2.povLeft().onTrue(new CandleCmd(m_candle, false));
        // m_driverController2.povRight().onTrue(new CandleCmd(m_candle, true));
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

        m_driverController3.leftTrigger()
            .and(m_driverController3.rightTrigger())
            .onTrue(new InstantCommand(() -> {
                GlobalConfig.devMode = !GlobalConfig.devMode;
            }));
    }

    private void registerPathplannerEventsAndNamedCommands() {
        UpperSystem2025Cmd cmd = UpperSystem2025Cmd.inst;
        long[] aprilTags = {17,18,19,20,21,22,6,7,8,9,10,11,};
        long[] levels = {1,2,3,4};

        for (long apId : aprilTags) {
            for (long level : levels) {
                // e.g. "17-L-1", "17-R-1", "18-L-2", "18-R-2", etc.
                String leftName = apId + "-L-" + level ;
                String rightName = apId + "-R-" + level ;

                
                // regPPEnC(leftName, cmd.getMoveToCoralCmdGroup(apId, level-1, -1, true));
                // regPPEnC(rightName, cmd.getMoveToCoralCmdGroup(apId, level-1, 1, false));

                regPPEnC(leftName, cmd.getMoveToCoralAndElevatorToLnCmd(apId, level-1, -1));
                regPPEnC(rightName, cmd.getMoveToCoralAndElevatorToLnCmd(apId, level-1, 1));
            }
        }

        regPPEnC("Intake", () -> {
            cmd.startIntake();
        });
        regPPEnC("Shoot", () -> {
            cmd.startShoot();
        });

        regPPEnC("GroundIntake", () -> {
           cmd.setIntakeSource(true);
        });

        regPPEnC("UpperIntake", () -> {
            cmd.setIntakeSource(false);
         });

        regPPEnC("ExpendGroundIntake", ()->{
            cmd.expendGroundIntake();
        });

        regPPEnC("ResetGroundIntake", ()->{
            cmd.makeSureGroundIntakeRetracted();
        });

        regPPEnC("FaceGroundCoral", new FaceGroundCoralCmd(drivetrain));
        regPPEnC("Go", new GoForward(drivetrain, cmd, 2));

        regPPEnC("WaitForShoot", new WaitForShootOverCmd(m_intake));

        regPPEnC("FakeMove", new FakeGoToCoralAndElevatorToLnCmd(drivetrain, cmd));

        // PathConstraints constraints = new PathConstraints(
        //     Constants.PathPlanner.constraintsSpeed, Constants.PathPlanner.constraintsAccel,
        //     Units.degreesToRadians(540), Units.degreesToRadians(720));

        // try {
        //     String[] ppPaths = GlobalConfig.loadAllPathPlannerPath();
        //     for (int i = 0; i < ppPaths.length; ++i) {
        //         String pathName = ppPaths[i];
        //         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        //         regPPEnC(pathName, AutoBuilder.pathfindThenFollowPath(path, constraints));
        //         // regPPEnC(pathName, ()->{
        //         //     System.out.println("===============================> regPPEnC: " + pathName);
        //         // });
        //     }
        // }
        // catch (Exception e) {
        //     System.err.println("Error loading PathPlanner paths: " + e.getMessage());
        //     e.printStackTrace();
        // }
    }

    private void regPPEnC(String name, Runnable runnable) {
        NamedCommands.registerCommand(name, new InstantCommand(runnable));
        pathplannerEvents.add(new EventTrigger(name).onTrue(new InstantCommand(runnable)));
    }

    private void regPPEnC(String name, Command cmd) {
        NamedCommands.registerCommand(name, cmd);
        pathplannerEvents.add(new EventTrigger(name).onTrue(cmd));
    }

    public Command getAutonomousCommand() {
        return m_autoPath;
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
        UpperSystem2025Cmd.inst.setShutdownAutoMoveToSource(false);
    }

    public void autoInit() {
        LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 0);
        LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 0);
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 0);

        UpperSystem2025Cmd.inst.schedule();
        UpperSystem2025Cmd.inst.setShutdownAutoMoveToSource(true);
        // UpperSystem2025Cmd.inst.getRequirements().forEach(sys -> {
        //     System.out.println("up: " + sys.getName());
        // });
    }

    public void testInit() {
        configureDriver3Bindings();
        UpperSystem2025Cmd.inst.schedule();
    }

    public void onDisabled() {

        SmartDashboardEx.flush();
        if (GlobalConfig.devMode) {
            LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 2);
            LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 2);
            // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 2);
        }
    }
}
