package frc.robot.Autonomus;

import static frc.robot.Autonomus.AutonomousProgram.create;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.RamseteAutoBuilder;
import frc.robot.Robot;
import frc.robot.Subsystems.DriveSubsystem;

// import org.json.simple.parser.JSONParser;
// import org.json.simple.JSONObject;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import org.json.simple.JSONArray;
import java.io.FileReader;
// import org.json.simple.JSONArray;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Robot;


/**
 * Quick guide to Comand Groups:
 *
 * SequentialComandGroup:
 * Will run all comands in order within it's parentheses
 * Note: If a comand does not have a isFinshed statment the code will be stuck
 * on that command forever
 *
 * ParallelCommandGroup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Both commands will have to finish to move on
 *
 * ParallelRaceGoup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: As soon as one command runs it's isfinished method runs then both
 * commands will end
 *
 * ParallelDeadlineGroup
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Only the first command will finish the group
 */

public class Autonomous {

    // private static RamseteAutoBuilder autoBuilder;


    public static void init() {
        HashMap<String, Command> eventMap = new HashMap<>();

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        /*
        autoBuilder = new RamseteAutoBuilder(
            tank::getPose, // Pose2d supplier
            tank::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            tank.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            tank::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            tank // The drive subsystem. Used to properly set the requirements of path following commands
        );
        */
        
    }
}
