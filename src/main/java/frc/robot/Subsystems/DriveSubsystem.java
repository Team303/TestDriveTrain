package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.io.IOException;
import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap.DDrive;
import frc.robot.RobotMap.DrivebaseConstants;
import frc.robot.RobotMap.PhotonvisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Modules.PhotonvisionModule.CameraName;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

// import static Robot.photonvision;

public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax m_frontLeftMotor;
    private final CANSparkMax m_frontRightMotor;
    private final CANSparkMax m_backLeftMotor;
    private final CANSparkMax m_backRightMotor;
    private final MotorController leftSideGroup;
    private final MotorController rightSideGroup;
    private final ShuffleboardTab DBSTab = Shuffleboard.getTab("Drive Base Subsystem");
    private final GenericEntry setpoint;
    private final GenericEntry output;
    private final GenericEntry error;
    private final GenericEntry angle;
    private final GenericEntry velocity;
    private final RelativeEncoder left;
    private final RelativeEncoder right;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.508);
    // private final DifferentialDriveOdometry dDriveOdometry;
    private final DifferentialDrive drive;

    public final AprilTagFieldLayout aprilTagField;
    private final Field2d field2d = new Field2d();
    private static final Vector<N3> differentialStandardDeviations = VecBuilder.fill(0,0, 0);
    // private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(0.25, 0.25, 0);
    private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(100000, 1000000, 1000000);


    public PhotonPoseEstimator visionPoseEstimatorFront;
    public PhotonPoseEstimator visionPoseEstimatorRight;
    public PhotonPoseEstimator visionPoseEstimatorBack;
    public PhotonPoseEstimator visionPoseEstimatorLeft;

    public DifferentialDrivePoseEstimator poseEstimator;

    // private Pose2d pose = new Pose2d(DDrive.STARTING_X, DDrive.STARTING_Y, new Rotation2d());;

    public DifferentialDrive getDrive() {
        return drive;
    }

    public DriveSubsystem() {
        m_frontLeftMotor = new CANSparkMax(DrivebaseConstants.FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
        m_frontRightMotor = new CANSparkMax(DrivebaseConstants.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
        m_backLeftMotor = new CANSparkMax(DrivebaseConstants.BACK_LEFT_SPARK_ID, MotorType.kBrushless);
        m_backRightMotor = new CANSparkMax(DrivebaseConstants.BACK_RIGHT_SPARK_ID, MotorType.kBrushless);

        leftSideGroup = new MotorControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        rightSideGroup = new MotorControllerGroup(m_frontRightMotor, m_backRightMotor);

        m_frontLeftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_frontRightMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_backLeftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_backRightMotor.setIdleMode(DrivebaseConstants.BRAKE);

        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);
        m_backLeftMotor.setInverted(true);
        m_backRightMotor.setInverted(true);
        

        drive = new DifferentialDrive(leftSideGroup, rightSideGroup);

        right = m_frontRightMotor.getEncoder();
        left = m_backLeftMotor.getEncoder();
        right.setPositionConversionFactor( 1/4.67);
        left.setPositionConversionFactor( 1/4.67);

        // dDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
        //         left.getPosition(),
        //         right.getPosition(),
        //         new Pose2d(DDrive.STARTING_X, DDrive.STARTING_Y, new Rotation2d()));

        output = DBSTab.add("PID Output", 0).getEntry();
        setpoint = DBSTab.add("PID Setpoint", 0).getEntry();
        error = DBSTab.add("Error", 0).getEntry();
        angle = DBSTab.add("Angle", 0).getEntry();
        velocity = DBSTab.add("Velocity", 0).getEntry();

        AprilTagFieldLayout initialLayout;

        try {
            initialLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            System.out.println(alliance.name());
            initialLayout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
                    : OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            initialLayout = null;
        }

        aprilTagField = initialLayout;
        if (Robot.isReal()) {
            visionPoseEstimatorFront = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP,
                    Robot.photonvision.getCamera(CameraName.CAM1),
                    PhotonvisionConstants.ROBOT_TO_FRONT_CAMERA);
            visionPoseEstimatorRight = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP,
                    Robot.photonvision.getCamera(CameraName.CAM2),
                    PhotonvisionConstants.ROBOT_TO_RIGHT_CAMERA);
            visionPoseEstimatorBack = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP,
                    Robot.photonvision.getCamera(CameraName.CAM3),
                    PhotonvisionConstants.ROBOT_TO_BACK_CAMERA);
            visionPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP,
                    Robot.photonvision.getCamera(CameraName.CAM4),
                    PhotonvisionConstants.ROBOT_TO_LEFT_CAMERA);
            visionPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            visionPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            visionPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            visionPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


        }
        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics,
                Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
                left.getPosition(),
                right.getPosition(),
                new Pose2d(DDrive.STARTING_X, DDrive.STARTING_Y, new Rotation2d()),
                differentialStandardDeviations,
                photonStandardDeviations);
        DBSTab.add("Pose", toString()).withPosition(0, 0).withSize(2, 0);
        DBSTab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    public MotorController getLeftSideGroup() {
        return leftSideGroup;
    }

    public MotorController getRightSideGroup() {
        return rightSideGroup;
    }

    public void arcadeDrive(double arcadeDriveSpeed, double arcadeDriveRotations) {
        getDrive().arcadeDrive(arcadeDriveSpeed, arcadeDriveRotations);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chassisSpeeds);
        tankDrive(speeds.leftMetersPerSecond / 4, speeds.rightMetersPerSecond / 4);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        getDrive().tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive(double xSpeed, double zRotations, boolean allowTurnInPlace) {
        getDrive().curvatureDrive(xSpeed, zRotations, allowTurnInPlace);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimatorFront.setReferencePose(prevEstimatedRobotPose);
        return visionPoseEstimatorFront.update();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseRight(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimatorRight.setReferencePose(prevEstimatedRobotPose);
        return visionPoseEstimatorRight.update();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimatorBack.setReferencePose(prevEstimatedRobotPose);
        return visionPoseEstimatorBack.update();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLeft(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimatorLeft.setReferencePose(prevEstimatedRobotPose);
        return visionPoseEstimatorLeft.update();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(left.getVelocity(), right.getVelocity());
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    @Override
    public void periodic() {

        Rotation2d gyroAngle = Rotation2d.fromDegrees(Robot.getNavX().getAngle());
        // pose = dDriveOdometry.update(gyroAngle,
        //         left.getPosition(),
        //         right.getPosition());

        Optional<EstimatedRobotPose> resultFront = getEstimatedGlobalPoseFront(poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> resultBack = getEstimatedGlobalPoseBack(poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> resultRight = getEstimatedGlobalPoseRight(poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> resultLeft = getEstimatedGlobalPoseLeft(poseEstimator.getEstimatedPosition());


        // if (resultFront.isPresent()) {
        //     EstimatedRobotPose visionPoseEstimate = resultFront.get();
        //     poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
        //             visionPoseEstimate.timestampSeconds);
        // }
        // if (resultRight.isPresent()) {
        //     EstimatedRobotPose visionPoseEstimate = resultRight.get();
        //     poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
        //             visionPoseEstimate.timestampSeconds);
        // }
        // if (resultBack.isPresent()) {
        //     EstimatedRobotPose visionPoseEstimate = resultBack.get();
        //     poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
        //             visionPoseEstimate.timestampSeconds);
        // }
        // if (resultLeft.isPresent()) {
        //     EstimatedRobotPose visionPoseEstimate = resultLeft.get();
        //     poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
        //             visionPoseEstimate.timestampSeconds);
        // }

        field2d.setRobotPose(getPose());
        poseEstimator.update(
                gyroAngle,
                left.getPosition(),
                right.getPosition());
        Robot.logger.recordOutput("Pose", poseEstimator.getEstimatedPosition());


        // System.out.println(poseEstimator.getEstimatedPosition());

        // if(result.isPresent()){
        // Robot.logger.recordOutput("Vision
        // Estimate",result.get().estimatedPose.toPose2d());
        // }

        angle.setDouble(Robot.getNavX().getAngle());

        double velo = Math
                .sqrt(Math.pow(Robot.getNavX().getVelocityX(), 2) + Math.pow(Robot.getNavX().getVelocityZ(), 2));
        if (velo > velocity.getDouble(velo)) {
            velocity.setDouble(velo);
        }

    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(Robot.getNavX().getRotation2d(), left.getPosition(), right.getPosition(),
                pose);
        Robot.getNavX().reset();
    }

    public void resetPose() {
        poseEstimator.resetPosition(Robot.getNavX().getRotation2d(), left.getPosition(), right.getPosition(),
                new Pose2d());
        Robot.getNavX().reset();
    }

    // public Command getPathFromFile(String pathName) {
    // PathPlannerTrajectory path = PathPlanner.loadPath("pathName", new
    // PathConstraints(2, 2));

    // return Robot.autoBuilder.followPathWithEvents(path);
    // }
}
