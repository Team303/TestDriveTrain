// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.ControllerConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.PIDForwardCommand;
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
public class Robot extends TimedRobot {

	/* RoboRio Sensors */
	private static final AHRS navX = new AHRS();

	public static AHRS getNavX() {
		return navX;
	}

	private static final DriveSubsystem tank = new DriveSubsystem();

	public static DriveSubsystem getDrivebase() {
			return tank;
	}

	public static final XboxController XBOX_CONTROLLER = new XboxController(ControllerConstants.CONTROLLER_ID);

	public void robotInit() {
		// We need to invert one side of the drivetrain so that positive voltages
		// result in both sides moving forward. Depending on how your robot's
		// gearbox is constructed, you might have to invert the left side instead.
		tank.getLeftSideGroup().setInverted(true);

		//Robot.XBOX_CONTROLLER.get

		if(Robot.XBOX_CONTROLLER.getLeftBumper()) {
			System.out.println("Pid mode");
			Robot.getDrivebase().setDefaultCommand(new PIDForwardCommand(0.9, .05, 0));
		}
		else {
			System.out.println("Normal mode");
			//Robot.tank.setDefaultCommand(new DefaultDrive());
			Robot.getDrivebase().setDefaultCommand(new PIDForwardCommand(0.9, .05, 0));

		}
		
		SmartDashboard.putBoolean("garmadon", Robot.XBOX_CONTROLLER.getLeftBumperPressed());

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
