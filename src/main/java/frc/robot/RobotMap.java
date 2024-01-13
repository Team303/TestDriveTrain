// Copyright (c) 2022 Team 303

package frc.robot;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class RobotMap {

	public static final class DrivebaseConstants {

		/* CAN IDs of the Motors on the Drive Base */
		public static final int FRONT_LEFT_SPARK_ID = 18;
		public static final int FRONT_RIGHT_SPARK_ID = 5;
		public static final int BACK_LEFT_SPARK_ID = 6;
		public static final int BACK_RIGHT_SPARK_ID = 7;
		public static final boolean LEFT_SPARK_INVERTED = true;
		public static final boolean RIGHT_SPARK_INVERTED = true;
		public static final IdleMode BRAKE = IdleMode.kBrake;
    }

	public static final class ControllerConstants {
		public static final int CONTROLLER_ID = 1;
		public static final int LEFT_CONT_ID = 0;
		public static final int RIGHT_CONT_ID = 1;
	}

	public static final class PhotonvisionConstants {
		public static final double FRONT_CAMERA_HEIGHT_METERS = 0.1778; // NOT FINAL
		public static final double BACK_CAMERA_HEIGHT_METERS = 0.14;
		public static final double RIGHT_CAMERA_HEIGHT_METERS = 0; //TODO: Measuring
		public static final double LEFT_CAMERA_HEIGHT_METERS = 0;
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double CAMERA_PITCH_RADIANS = 0; // NOT FINAL
		
		public static final Transform3d ROBOT_TO_FRONT_CAMERA= new Transform3d(new Translation3d(0.406, 0, FRONT_CAMERA_HEIGHT_METERS),new Rotation3d(0.0,0.0,0.0));
		public static final Transform3d ROBOT_TO_BACK_CAMERA= new Transform3d(new Translation3d(-0.4571,0,BACK_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(180),0));
		//TODO: Measuring
		public static final Transform3d ROBOT_TO_LEFT_CAMERA= new Transform3d(new Translation3d(0,0.2794,LEFT_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(90),0));
		public static final Transform3d ROBOT_TO_RIGHT_CAMERA= new Transform3d(new Translation3d(0,-0.2794,RIGHT_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(270),0));

	}
	public static final class DDrive {
		public static final double STARTING_X=0;
		public static final double STARTING_Y=0;
	}


}
