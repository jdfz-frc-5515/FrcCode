package frc.robot.subsystems.Chassis;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.SmartDashboardEx;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

import frc.robot.Library.MUtils.SegmentOnTheField;
import frc.robot.Library.team1706.MathUtils;
import frc.robot.Library.MUtils;
import edu.wpi.first.math.MathUtil;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.W
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    /** Swerve request to apply during field-centric move to Pose */
    private static final SwerveRequest.ApplyFieldSpeeds m_moveToPoseDrive = new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    
    PIDController m_translationXController, m_translationYController, m_rotationController;
    Pose2d m_targetPose2d = new Pose2d();
    double m_distFromTarget = Double.MAX_VALUE;
    double m_angleDistFromTarget = Double.MAX_VALUE;
    /** Swerve request to apply during Manual drivemode */
    private static double manual_MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double manual_MaxAngularRate = RotationsPerSecond.of(1.).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static final SwerveRequest.FieldCentric m_manualDrive = new SwerveRequest.FieldCentric().withDeadband(0.15).withRotationalDeadband(0.05 * manual_MaxAngularRate).withDriveRequestType(DriveRequestType.Velocity);

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> {
                SignalLogger.writeString("SysIdTranslation_State", state.toString());
            }
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        configureMoveToPose();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        configureMoveToPose();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
        configureMoveToPose();
    }


    
    private void configureAutoBuilder(){
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(Constants.AutoConstants.followPathTranslationkP, Constants.AutoConstants.followPathTranslationkI, Constants.AutoConstants.followPathTranslationkD),
                    // PID constants for rotation
                    new PIDConstants(Constants.AutoConstants.followPathRotationkP, Constants.AutoConstants.followPathRotationkI, Constants.AutoConstants.followPathRotationkD)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> false,  // DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    private void configureMoveToPose() {

        m_translationXController = new PIDController(Constants.AutoConstants.moveToPoseTranslationkP,
                Constants.AutoConstants.moveToPoseTranslationkI, Constants.AutoConstants.moveToPoseTranslationkD);
        m_translationYController = new PIDController(Constants.AutoConstants.moveToPoseTranslationkP,
                Constants.AutoConstants.moveToPoseTranslationkI, Constants.AutoConstants.moveToPoseTranslationkD);
        m_rotationController = new PIDController(Constants.AutoConstants.moveToPoseRotationkP, Constants.AutoConstants.moveToPoseRotationkI,
                Constants.AutoConstants.moveToPoseRotationkD);

        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        // m_rotationController.setTolerance(Constants.AutoConstants.moveToPoseRotationToleranceRadians,Constants.AutoConstants.moveToPoseAngularVelocityToleranceRadsPerSecond);
        // m_translationXController.setTolerance(Constants.AutoConstants.moveToPoseTranslationToleranceMeters, Constants.AutoConstants.moveToPoseVelocityToleranceMetersPerSecond);
        // m_translationYController.setTolerance(Constants.AutoConstants.moveToPoseTranslationToleranceMeters, Constants.AutoConstants.moveToPoseVelocityToleranceMetersPerSecond);

        m_rotationController.setTolerance(Constants.AutoConstants.moveToPoseRotationToleranceRadians);
        m_translationXController.setTolerance(Constants.AutoConstants.moveToPoseTranslationToleranceMeters);
        m_translationYController.setTolerance(Constants.AutoConstants.moveToPoseTranslationToleranceMeters);

    }


    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        double dist = updateOdometry(Constants.LIME_LIGHT_ARPIL_TAG_NAME_SKY, -1);
        if (MiscUtils.compareDouble(dist, 0)) {
            dist = -1;
        }
        updateOdometry(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, dist);
        updateOdometry(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, dist);
        
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }

    public ChassisSpeeds getSpeeds(){
        return this.getState().Speeds;
    }

    public SwerveModuleState[] getModuleStates() {
        return this.getState().ModuleStates;
    }

    public void driveFieldCentric(ChassisSpeeds speeds){
        driveFieldCentric(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public void driveFieldCentric(double vx, double vy, double omegaRadsPerSec){
        this.setControl(m_manualDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omegaRadsPerSec));
    }

    public void driveFieldCentric(ImprovedCommandXboxController controller){
        driveFieldCentric(
            -MathUtils.signedPow(MathUtil.applyDeadband(controller.getLeftY(), 0.1), 1.3) * manual_MaxSpeed,
            -MathUtils.signedPow(MathUtil.applyDeadband(controller.getLeftX(), 0.1), 1.3) * manual_MaxSpeed,
            -controller.getRightX() * manual_MaxAngularRate
        );
    }

    static final double MAX_LL_LATENCY = 100; // 100 ms, this is the maximum latency we accept from the limelight
    // 如果LL看到目标，则返回到目标的距离
    // 参数minDist表示当前看到的目标距离大于这个值，则忽略此目标，负数则无效
    public double updateOdometry(String llName, double minDist){

        double botRot = getState().Pose.getRotation().getDegrees();
        double pigeonRot = getRotationFromPigeon();

        // Pose2d botPos =  LimelightHelpers.getBotPose2d(llName);
        // SmartDashboardEx.putString("ROT-MT1", String.format("%f", botPos.getRotation().getDegrees()));
        // SmartDashboardEx.putString( "ROT", String.format("%f - %f - %f", botRot, pigeonRot, pigeonRot - zeroOdoDegree));
        LimelightHelpers.SetRobotOrientation(llName, getRotationFromPigeon() - zeroOdoDegree, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
        // ImprovedLL.MT2stddevs devs = ImprovedLL.getmt2Devs();
        // ImprovedLL.mt2stdDev stdDev = ImprovedLL.getmt2Dev(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT); 
        if(mt2 == null) {
            // DriverStation.reportWarning(llName + " Diconnected!", false);
            return -1;
        }
        
        if (minDist > 0 && mt2.avgTagDist > minDist) {
            return -1;
        }
        // SmartDashboardEx.putNumber(llName+"-DIST", mt2.avgTagDist);
        if (Math.abs(getSpeeds().omegaRadiansPerSecond) <= 4*Math.PI 
            && mt2.tagCount > 0 
            && mt2.avgTagDist < 4 
            && Math.hypot(getSpeeds().vxMetersPerSecond, getSpeeds().vyMetersPerSecond) < 2
            // && mt2.latency < MAX_LL_LATENCY // 抛弃高延时
        ) {

            double captureTime2 = Utils.fpgaToCurrentTime(mt2.timestampSeconds);

            double data = Constants.PoseEstimatorConstants.tAtoDev.get(mt2.avgTagArea);
            // SmartDashboardEx.putString("MT2 pos: ", mt2.pose.toString());
            addVisionMeasurement(mt2.pose,
                captureTime2,
                VecBuilder.fill(data, data, 100000000)
                // VecBuilder.fill(0.5,0.5, 100000000.)
                // VecBuilder.fill(devs.xdev, devs.ydev, 100000000.)
            );

            // SmartDashboardEx.putNumber("tA", mt2.avgTagArea );
            // SmartDashboardEx.putNumber("Dev", data);

            return mt2.avgTagDist;
        }
        else {
            if (mt2.latency >= MAX_LL_LATENCY) {
                DriverStation.reportWarning(llName + " latency too high: " + mt2.latency + " ms", false);
            }
            else {
                DriverStation.reportWarning(llName + "vision ignored", false);
            }
        }
        return -1;
        
    }

    public boolean isAtTranslation(Translation2d t){
        return getPose().getTranslation().minus(t).getNorm() <= Constants.AutoConstants.moveToPoseTranslationToleranceMeters;
    }

    public boolean isAtRotation(Rotation2d r){
        return Math.abs(getPose().getRotation().minus(r).getRadians()) <= Constants.AutoConstants.moveToPoseRotationToleranceRadians;
    }

    public boolean isAtPose(Pose2d pose){
        return isAtTranslation(pose.getTranslation()) && isAtRotation(pose.getRotation());
    }

    public boolean isAtTargetPose(){
        // return isAtPose(m_targetPose2d);
        Translation2d t = m_targetPose2d.getTranslation();
        Rotation2d r = m_targetPose2d.getRotation();
        double dist = getPose().getTranslation().minus(t).getNorm();
        double angleDist = Math.abs(getPose().getRotation().minus(r).getRadians());
        m_distFromTarget = dist;
        m_angleDistFromTarget = angleDist;
        return (dist <= Constants.AutoConstants.moveToPoseTranslationToleranceMeters) &&
            (angleDist <= Constants.AutoConstants.moveToPoseRotationToleranceRadians);
    }

    public double getDistFromTargetPose(){
        return m_distFromTarget;
    }

    public double getAngleDistFromTargetPose(){
        return m_angleDistFromTarget;
    }
    
    /** Caution this would reset your targetPose to this pose */
    public ChassisSpeeds getAutoMoveToPoseSpeeds(Pose2d pose){
        m_targetPose2d = pose;
        Pose2d currentPose = getPose();
        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        return optimizeMoveToSpeeds(optimizeMoveToSpeeds(
            new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity)
        ));
    }

    public ChassisSpeeds getAutoMoveToPoseSpeeds(Pose2d pose, double maxMoveToSpeed){
        m_targetPose2d = pose;
        Pose2d currentPose = getPose();
        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        return optimizeMoveToSpeeds(optimizeMoveToSpeeds(
            new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity), maxMoveToSpeed
        ));
    }

    public void customMoveWithSpeed(double xSpeed, double ySpeed) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        request = request.withVelocityX(xSpeed);
        request = request.withVelocityY(ySpeed);
        this.setControl(request);
    }
    public void customRotate(double angularSpeed) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        request = request.withRotationalRate(angularSpeed).withVelocityX(0).withVelocityY(0);
        this.setControl(request);
    }

    public void customStopMoving(boolean doIt) {
        if (!doIt) {
            return;
        }
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        request = request.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        this.setControl(request);
    }

    public Command autoMoveToPoseCommand(Pose2d pose){
        return this.applyRequest(() -> m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose)));
    }

    public void autoMoveToPose(Pose2d pose){

        this.setControl(m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose)));
    }

    public void autoMoveToPose(Pose2d pose, double maxSpeed) {
        this.setControl(m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose, maxSpeed)));
    }

    public void setFieldCentric() {
        // getPose().getRotation().getDegrees();
        setOperatorPerspectiveForward(getPose().getRotation());
    }

    private ChassisSpeeds optimizeMoveToSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds optimizedSpd = speeds;

        //If too small, apply deadband;
        // optimizedSpd.vxMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vxMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        // optimizedSpd.vyMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vyMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        // optimizedSpd.omegaRadiansPerSecond = MathUtil.applyDeadband(optimizedSpd.omegaRadiansPerSecond, AutoConstants.moveToAnguVeloDeadband, 1e6);

        //if too big, apply limit;
        if(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > Constants.AutoConstants.maxMoveToSpeed) {
            optimizedSpd = optimizedSpd.times(Constants.AutoConstants.maxMoveToSpeed / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        }
        if(Math.abs(speeds.omegaRadiansPerSecond) > Constants.AutoConstants.maxMoveToAngularVelocity) {
            optimizedSpd.omegaRadiansPerSecond = Math.copySign(Constants.AutoConstants.maxMoveToAngularVelocity, speeds.omegaRadiansPerSecond);
        }
        return optimizedSpd;
    }


    private ChassisSpeeds optimizeMoveToSpeeds(ChassisSpeeds speeds, double maxMoveToSpeed) {
        ChassisSpeeds optimizedSpd = speeds;

        //If too small, apply deadband;
        // optimizedSpd.vxMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vxMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        // optimizedSpd.vyMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vyMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        // optimizedSpd.omegaRadiansPerSecond = MathUtil.applyDeadband(optimizedSpd.omegaRadiansPerSecond, AutoConstants.moveToAnguVeloDeadband, 1e6);

        //if too big, apply limit;
        if(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > maxMoveToSpeed) {
            optimizedSpd = optimizedSpd.times(maxMoveToSpeed / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        }
        if(Math.abs(speeds.omegaRadiansPerSecond) > Constants.AutoConstants.maxMoveToAngularVelocity) {
            optimizedSpd.omegaRadiansPerSecond = Math.copySign(Constants.AutoConstants.maxMoveToAngularVelocity, speeds.omegaRadiansPerSecond);
        }
        return optimizedSpd;
    }

    /** Caution this would reset your targetPose to this pose */
    public ChassisSpeeds getHybridMoveToPoseSpeeds(Pose2d pose, ImprovedCommandXboxController driverController, double translationAdjustmentRange, double rotationAdjustmentRangeDegs){
        Pose2d currentPose = getPose();

        //Hybrid Control, read the lines carefully before you tune it
        Transform2d adjustment = new Transform2d(
            driverController.getLeftY()*translationAdjustmentRange * (DriverStation.getAlliance().get().equals(Alliance.Blue)? 1. : -1.), 
            driverController.getLeftX()*translationAdjustmentRange * (DriverStation.getAlliance().get().equals(Alliance.Blue)? 1. : -1.),
            Rotation2d.fromDegrees(-driverController.getRightX()*rotationAdjustmentRangeDegs)
        );
        pose = pose.plus(adjustment);
        m_targetPose2d = pose;

        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        if(isAtTranslation(pose.getTranslation())){
            targetTranslationXVelocity = 0.;
            targetTranslationYVelocity = 0.;
        }
        if(isAtRotation(pose.getRotation())) {
            targetAngularVelocity = 0.;
        }

        // SmartDashboardEx.putNumber("targetXVeloUnoptimized", targetTranslationXVelocity);
        // SmartDashboardEx.putNumber("targetYVeloUnoptimized", targetTranslationYVelocity);
        // SmartDashboardEx.putNumber("targetAngularVeloUnoptimized", targetAngularVelocity);

        return optimizeMoveToSpeeds(
            new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity)
        );
    }

    public Command hybridMoveToPoseCommand(Supplier<Pose2d> pose, ImprovedCommandXboxController driverController, double translationAdjustmentRange, double rotationAdjustmentRangeDegs) { // TODO
        return this.applyRequest(() -> m_moveToPoseDrive.withSpeeds(getHybridMoveToPoseSpeeds(pose.get(), driverController, translationAdjustmentRange, rotationAdjustmentRangeDegs)));
    }

    public void hybridMoveToPose(Pose2d pose, ImprovedCommandXboxController driverController, double translationAdjustmentRange, double rotationAdjustmentRangeDegs) {
        this.setControl(m_moveToPoseDrive.withSpeeds(getHybridMoveToPoseSpeeds(pose, driverController, translationAdjustmentRange, rotationAdjustmentRangeDegs)));
    }

    public PathPlannerPath generateChoreoPath(String pathName){
        try {
            return PathPlannerPath.fromChoreoTrajectory(pathName);
        } catch (Exception e) {
            e.printStackTrace();
        } 
        return null;
    }

    public PathPlannerPath generatePPPath(String pathName){
        try{ 
            return PathPlannerPath.fromPathFile(pathName);
        }catch(Exception e){
            e.printStackTrace();
            return null;
        }
    }

    public Command followChoreoPath(String pathName){
        return AutoBuilder.followPath(generateChoreoPath(pathName));
    }

    public Command followPPPath(String pathName){
        return AutoBuilder.followPath(generatePPPath(pathName));
    }

    // public Pose2d generateReefPose(int index){
    //     Translation2d t = FieldConstants.BlueReefCenterPos;
    //     Translation2d dt = FieldConstants.DReefTranslation12;
    //     if(index % 2 == 1){
    //         dt = new Translation2d(dt.getX(), -dt.getY());
    //     }
    //     t = t.plus(dt);

    //     Rotation2d r = new Rotation2d(Math.PI);
    //     Rotation2d dr = Rotation2d.fromDegrees(
    //             (double)((index+1) / 2) * 60.
    //     );

    //     t = t.rotateAround(FieldConstants.BlueReefCenterPos, dr);
    //     r = r.plus(dr);

    //     Pose2d res = new Pose2d(t, r);

    //     if(DriverStation.getAlliance().get() == Alliance.Red){
    //         // t = t.rotateAround(FieldConstants.FieldCenter, Rotation2d.fromDegrees(180));
    //         // r = r.plus(Rotation2d.fromDegrees(180));
    //         res = FieldConstants.rotateAroundCenter(res, FieldConstants.FieldCenter, Rotation2d.k180deg);
    //     }

    //     if(res == null){
    //         DriverStation.reportWarning("1111111111111111111", false);
    //     }

    //     return res;
    // }

    // public Translation2d getToReefCentreTranslation(){
    //     // Translation2d t = FieldConstants.BlueReefCenterPos.minus(getPose().getTranslation());
    //     // if(DriverStation.getAlliance().get() == Alliance.Red){
    //     //     t.rotateBy(new Rotation2d(Math.PI));
    //     // }
    //     // return t;

    //     return DriverStation.getAlliance().get() == Alliance.Blue?
    //         FieldConstants.BlueReefCenterPos.minus(getPose().getTranslation()) :
    //         FieldConstants.BlueReefCenterPos.rotateAround(FieldConstants.FieldCenter, Rotation2d.k180deg).minus(getPose().getTranslation())
    //     ;

    // }

    /**
     * Automatically generates a pose for station based on the current pose.
     * <p>Only processes blue originated inputs! Which means if you're on Red, rotate the input around the center.
     * <p>The algorithm is as the following:
     * <p>1. Define 2 segments in the field representing the Left and Right Station, naming L and R.
     * <p>2. Determine which segment on which we should land, based on the current pose.
     * <p>3. Finds the nearest point on the segment to the current pose, and generates a pose that is at that point.
     * <p>     - If the projection of the current pose onto the segment is on the segment, then the pose is at that point.
     * <p>     - If the projection of the current pose onto the segment is outside the segment, then the pose is at the closest point on the segment to the current pose.
     * 
     * @param currentPose the current pose of the robot, blue origin
     * @return the 
     */

    // public Pose2d generateStationPose(Pose2d currentPose){
    //     SegmentOnTheField targetSegment = currentPose.getY() >= FieldConstants.FieldCenter.getY() ? FieldConstants.BlueLeftStation : FieldConstants.BlueRghtStation;
    //     Translation2d t = targetSegment.getNearestPointTo(currentPose.getTranslation());
    //     Rotation2d r = Rotation2d.fromRadians(Math.atan(-1. / targetSegment.k));
    //     return new Pose2d(t, r);
    // }

    /**
     * Only accepts blue originated inputs! Which means if you're on Red, rotate the input around the center.
     * For more information, see {@link #generateStationPose(Pose2d)}
     * @param currentPose
     * @param distance
     * @return
     */
    // public Pose2d generateStationPose(Pose2d currentPose, double distance){
    //     SegmentOnTheField targetSegment = currentPose.getY() >= FieldConstants.FieldCenter.getY() ? FieldConstants.BlueLeftStation : FieldConstants.BlueRghtStation;
    //     distance = MUtils.numberLimit(0, targetSegment.getLength(), distance);
    //     Translation2d t = targetSegment.calculate(targetSegment.m + distance / Math.sqrt(targetSegment.k * targetSegment.k + 1.));
    //     Rotation2d r = Rotation2d.fromRadians(Math.atan(-1. / targetSegment.k));
    //     return new Pose2d(t, r);
    // }

    // public Pose2d generateStationPose(double distance){
    //     return generateStationPose(getPose(), distance);
    // }

    // public Pose2d generateStationPose(){
    //     if (DriverStation.getAlliance().get() == Alliance.Red){
    //         return FieldConstants.rotateAroundCenter(
    //             generateStationPose(FieldConstants.rotateAroundCenter(getPose(), FieldConstants.FieldCenter, Rotation2d.k180deg)),
    //             FieldConstants.FieldCenter,
    //             Rotation2d.k180deg
    //         );
    //     }
    //     return generateStationPose(getPose());
    // }

    // public int generateReefIndex(){
    //     Translation2d t = getToReefCentreTranslation();
    //     if(DriverStation.getAlliance().get() == Alliance.Red){
    //         t.rotateBy(new Rotation2d(Math.PI));
    //     }
    //     double angle = t.getAngle().getDegrees();
    //     int res = (int)Math.floor(angle / 30.) + 11;
    //     return res % 12 + 1;
    // }

    public enum AutonomousTargetType{
        BARGE,      //Should not be called unless you're returning to barge at teammates' request.
        STATION,
        REEF,
    }

    /**
     * This needs future maintenance.
     * @param type
     * @param index
     * @return
     */
    // public Pose2d generateAutonomousTargetPose(AutonomousTargetType type, int index){       //TODO
    //     switch(type){
    //         case BARGE:
    //             // return generateBargePose(index);     //TODO
    //             return new Pose2d();
    //         case STATION:
    //             return generateStationPose();
    //         case REEF:
    //             return generateReefPose(index);
    //         default:
    //             return new Pose2d();
    //     }
    // }

    public Command getPathFindToPoseCommand(Pose2d pose){
        return AutoBuilder.pathfindToPose(pose, Constants.AutoConstants.generatedPathCommandConstraints);
    }

    /**
     * Checks if the robot is near the given translation.
     * @param t in meters, blue origin
     * @param distance in meters
     * @return
     */
    public boolean isNearTranslation(Translation2d t, double distance){
        return getPose().getTranslation().getDistance(t) <= distance;
    }

    public void resetHeadingForOdo(double angle) {
        resetRotation(new Rotation2d(Units.degreesToRadians(angle)));
        zeroOdoDegree = getRotationFromPigeon() + angle;
        // this.resetRotation(Rotation2d.fromDegrees(angle));
    }


    private double zeroOdoDegree = 0;
    private double getRotationFromPigeon() {
        return this.getPigeon2().getYaw().getValueAsDouble() % 360;
    }

    // public boolean isNearStation(){
    //     return generateStationPose().getTranslation().getDistance(getPose().getTranslation()) <= FieldConstants.StationDetectionArea;
    // }

}
