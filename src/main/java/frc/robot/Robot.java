// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.ControllerConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.PIDForwardCommand;
import frc.robot.Modules.PhotonvisionModule;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotMap.DrivebaseConstants;
import frc.robot.Subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.lang.ModuleLayer.Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;
import frc.robot.Commands.DefaultDrive;

import com.kauailabs.navx.frc.AHRS;
/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends LoggedRobot {

	/* RoboRio Sensors */
	private static final AHRS navX = new AHRS();
	public static final PhotonvisionModule photonvision = new PhotonvisionModule();
	private static final DriveSubsystem tank = new DriveSubsystem();
	public static final Logger logger = Logger.getInstance(); 

	public static enum SubstationFiducialID {
		RED(5),
		BLUE(4);

		public final int fiducialId;

		private SubstationFiducialID(int fiducialId) {
			this.fiducialId = fiducialId;
		}
	}
	public static final int ALLIANCE_SUBSTATION_ID = DriverStation.getAlliance() == Alliance.Blue
			? SubstationFiducialID.RED.fiducialId
			: SubstationFiducialID.BLUE.fiducialId;

	public static AHRS getNavX() {
		return navX;
	}


	public static DriveSubsystem getDrivebase() {
			return tank;
	}

	public static final XboxController XBOX_CONTROLLER = new XboxController(ControllerConstants.CONTROLLER_ID);

	public void robotInit() {
		// We need to invert one side of the drivetrain so that positive voltages
		// result in both sides moving forward. Depending on how your robot's
		// gearbox is constructed, you might have to invert the left side instead.
		Logger logger = Logger.getInstance();
		navX.reset();
		CameraServer.startAutomaticCapture();

		// Record metadata
		logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Set up data receivers & replay source
		if (Robot.isReal()) {
			// Running on a real robot, log to a USB stick
			logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
			logger.addDataReceiver(new NT4Publisher());
		} else {
			logger.addDataReceiver(new WPILOGWriter(""));
			logger.addDataReceiver(new NT4Publisher());
		}

		tank.getLeftSideGroup().setInverted(true);


		tank.setDefaultCommand(new DefaultDrive());

		//Robot.XBOX_CONTROLLER.get

		// if(Robot.XBOX_CONTROLLER.getLeftBumper()) {
		// 	System.out.println("Pid mode");
		// 	// Robot.getDrivebase().setDefaultCommand(new PIDForwardCommand(0.9, .05, 0));
		// }
		// else {
		// 	System.out.println("Normal mode");
		// 	//Robot.tank.setDefaultCommand(new DefaultDrive());
		// 	Robot.getDrivebase().setDefaultCommand(new PIDForwardCommand(0.9, .05, 0));

		// }
		// SmartDashboard.putBoolean("garmadon", Robot.XBOX_CONTROLLER.getLeftBumperPressed());
		// Start Camera
		logger.start();
	}

  	@Override
	public void robotPeriodic() {

		/*
		 * Runs the Scheduler. This is responsible for polling buttons, adding
		 * newly-scheduled
		 * commands, running already-scheduled commands, removing finished or
		 * interrupted commands,
		 * and running subsystem periodic() methods. This must be called from the
		 * robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */
		CommandScheduler.getInstance().run();
		//System.out.print("Left Joystick: "); System.out.println(-XBOX_CONTROLLER.getLeftY());
		//System.out.print("Right Joystick: "); System.out.println(-XBOX_CONTROLLER.getRightY());
		
	}
}
