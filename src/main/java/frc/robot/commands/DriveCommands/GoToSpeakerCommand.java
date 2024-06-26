package frc.robot.commands.DriveCommands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive.Drive;

public class GoToSpeakerCommand extends Command {
        private final Drive drive;
        private CommandScheduler scheduler;

        public GoToSpeakerCommand(
                        Drive drive, CommandScheduler scheduler) {
                this.drive = drive;
                this.scheduler = scheduler;
                addRequirements(drive);
        }

        @Override
        public void initialize() {
                Pose2d targetPose = new Pose2d(4.25, 7.20, Rotation2d.fromDegrees(21.68));

                PathConstraints constraints = new PathConstraints(
                                3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

                Logger.recordOutput("GoToSpeaker/RobotPose", drive.getPose());
                Logger.recordOutput("GoToSpeaker/TargetPose", targetPose);

                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(targetPose, targetPose);

                PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints, new GoalEndState(0.0, Rotation2d.fromDegrees(21.68)));
                path.preventFlipping = true;

                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                        path,
                        constraints,
                        3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                );

                scheduler.schedule(pathfindingCommand);
        }

        @Override
        public void execute() {
        }

        @Override
        public void end(boolean interrupted) {

        }

        @Override
        public boolean isFinished() {
                return true;
        }

}
