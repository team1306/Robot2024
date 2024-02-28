package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.subsystems.vision.NoteDetector.Checkpoints;

public class FarRingsFromShootTop extends SequentialCommandGroup {
    public FarRingsFromShootTop(NoteDetector detector) {
        addCommands(
            //Collect 1 and 2
            detector.read(Checkpoints.TOP),
            detector.notePresenceCommandSwitcher(1,
                //if note 1 present
                new SequentialCommandGroup(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top")),
                    detector.read(Checkpoints.TOP),
                    detector.notePresenceCommandSwitcher(2,
                        //if note 2 present
                        new SequentialCommandGroup(
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")), 
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top"))
                        ),
                    //if note 2 not present
                        new InstantCommand()
                    )
                ),
                //if note 1 not present
                detector.notePresenceCommandSwitcher(2,
                    //if note 2 present
                    new SequentialCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")), 
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top"))
                    ),
                    //if note 2 not present
                    new InstantCommand()
                )
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Scan-Top"))

            //Collect 3, 4, and 5


        );
    }
}