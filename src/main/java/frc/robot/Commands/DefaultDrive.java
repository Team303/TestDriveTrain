package frc.robot.Commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * If no other commands are running, drive based on joystick inputs
 */
public class DefaultDrive extends CommandBase {
	public DefaultDrive() {
		addRequirements(Robot.getDrivebase());
	}

	@Override
	public void execute() {
		// Drive based on the joystick's y position (forward and back on ours)
		Robot.getDrivebase().arcadeDrive(-Robot.XBOX_CONTROLLER.getLeftY(),-Robot.XBOX_CONTROLLER.getRightY());
	}
}