package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.NoteDetector;

public class FarRingsFromCloseMiddle extends SequentialCommandGroup {
    public FarRingsFromCloseMiddle(NoteDetector detector) {
        addCommands(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-Middle to Far-1")),
            detector.notePresenceCommandSwitcher(
                1,
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-Middle to Far-1")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-Middle to Far-1"))
            )
        );
    }
}