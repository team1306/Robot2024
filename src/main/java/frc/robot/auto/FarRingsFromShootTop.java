package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.NoteDetector;

public class FarRingsFromShootTop extends SequentialCommandGroup {
    public FarRingsFromShootTop(NoteDetector detector) {
        addCommands(
            detector.read();
            detector.notePresenceCommandSwitcher(1,
                new SequentialCommandGroup(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top"))
                ),
                null
            ),
            
            
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-Middle to Far-1")),
            detector.notePresenceCommandSwitcher(
                1,
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-Middle to Far-1")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-Middle to Far-1"))
            )
        );
    }
}