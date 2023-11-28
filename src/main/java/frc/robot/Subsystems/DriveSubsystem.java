package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;


public class DriveSubsystem extends SubsystemBase{

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
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.635);
    private final DifferentialDriveOdometry dDriveOdometry; 
    private final DifferentialDrive drive;

    private Pose2d pose; 


    public DifferentialDrive getDrive() {
        return drive;
    }

    public DriveSubsystem()
     {
        m_frontLeftMotor = new CANSparkMax(DrivebaseConstants.FRONT_LEFT_SPARK_ID, MotorType.kBrushed);
        m_frontRightMotor = new CANSparkMax(DrivebaseConstants.FRONT_RIGHT_SPARK_ID, MotorType.kBrushed);
        m_backLeftMotor = new CANSparkMax(DrivebaseConstants.BACK_LEFT_SPARK_ID, MotorType.kBrushed);
        m_backRightMotor = new CANSparkMax(DrivebaseConstants.BACK_RIGHT_SPARK_ID, MotorType.kBrushed);
      
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

        right.setPositionConversionFactor(1/4.67);
        left.setPositionConversionFactor(1/4.67);

        dDriveOdometry = new DifferentialDriveOdometry(new Rotation2d(Robot.getNavX().getAngle()),
        left.getPosition(), 
        right.getPosition(), 
        new Pose2d(0, 0, new Rotation2d()));

        output = DBSTab.add("PID Output", 0).getEntry();
        setpoint = DBSTab.add("PID Setpoint", 0).getEntry(); 
        error = DBSTab.add("Error", 0).getEntry();
        angle = DBSTab.add("Angle", 0).getEntry();
        velocity = DBSTab.add("Velocity", 0).getEntry();
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
        tankDrive(speeds.leftMetersPerSecond/4, speeds.rightMetersPerSecond/4);
    }


    public void tankDrive(double leftSpeed, double rightSpeed) {
        getDrive().tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive(double xSpeed, double zRotations, boolean allowTurnInPlace) {
        getDrive().curvatureDrive(xSpeed, zRotations, allowTurnInPlace);
    }

    public Pose2d getPose() {
        return pose;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(left.getVelocity(), right.getVelocity());
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }
    
    @Override
    public void periodic() {

        Rotation2d gyroAngle = new Rotation2d(Math.toRadians(Robot.getNavX().getAngle()));

        pose = dDriveOdometry.update(gyroAngle,
        left.getPosition(),
        right.getPosition());

        Robot.logger.recordOutput("Odometry", pose);
        
        angle.setDouble(Robot.getNavX().getAngle());
        
        double velo = Math.sqrt(Math.pow(Robot.getNavX().getVelocityX(), 2) + Math.pow(Robot.getNavX().getVelocityZ(), 2));
        if (velo > velocity.getDouble(velo)) {
            velocity.setDouble(velo);
        }

    }

    public void resetPose(Pose2d pose) {
        dDriveOdometry.resetPosition(Robot.getNavX().getRotation2d(), left.getPosition(), right.getPosition(), new Pose2d());
        Robot.getNavX().reset();
    }

    public void resetPose() {
        dDriveOdometry.resetPosition(Robot.getNavX().getRotation2d(), left.getPosition(), right.getPosition(), new Pose2d());
        Robot.getNavX().reset();
    }

    // public Command getPathFromFile(String pathName) {
    //     PathPlannerTrajectory path = PathPlanner.loadPath("pathName", new PathConstraints(2, 2));
        
    //     return Robot.autoBuilder.followPathWithEvents(path);
    // }
}
