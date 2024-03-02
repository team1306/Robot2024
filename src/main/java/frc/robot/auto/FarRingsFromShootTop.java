package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.NoteDetector;

public class FarRingsFromShootTop extends SequentialCommandGroup {

    private static final SequentialCommandGroup far1 = new SequentialCommandGroup( //far-1 path stored as a variable
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top"))
            //shoot
    );

    private static final SequentialCommandGroup far2 = new SequentialCommandGroup( //far-1 path stored as a variable
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top"))
            //shoot
    );
    
    private static final SequentialCommandGroup far4 = new SequentialCommandGroup( //far-4 path stored as a variable
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-4")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom"))
            //shoot
    );

    private static final SequentialCommandGroup far5 = new SequentialCommandGroup( //far-5 path stored as a variable
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-5")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom"))
            //shoot
    );

    public FarRingsFromShootTop(NoteDetector detector) {
        addCommands(
            //Collect 1 and 2
            detector.read(),
            detector.notePresenceCommandSwitcher(1,
                //if note 1 present
                new SequentialCommandGroup(
                    far1, //collect far-1
                    detector.read(),
                    detector.notePresenceCommandSwitcher(2,
                        far2, //if note 2 is present collect it
                        new InstantCommand() //if note 2 is not present do nothing
                    )
                ),
                //if note 1 not present
                detector.notePresenceCommandSwitcher(2,
                    far2, //if note 2 is present collect it
                    //if note 2 not present
                    new InstantCommand() //if note 2 is not present do nothing
                )
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Scan-Top")), //Go to scan bottom rings

            //Collect 3, 4, and 5
            detector.read(),
            detector.notePresenceCommandSwitcher(3,
            //if note 3 is present
            new SequentialCommandGroup( //get note 3 and go to Shoot-Bottom
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-3")),
                //intake
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Bottom")),
                //shoot
                detector.read(),
                detector.notePresenceCommandSwitcher(4,
                    //if note 4 is present
                    new SequentialCommandGroup(
                        far4,
                        detector.read(),
                        detector.notePresenceCommandSwitcher(5,
                            far5, //if note 5 is present collect it
                            new InstantCommand() //if note 5 is not present do nothing
                        )
                    ),
                    //if note 4 is not present
                    detector.notePresenceCommandSwitcher(5,
                        far5, //if note 5 is present collect it
                        new InstantCommand() //if note 5 is not present do nothing
                    )
                )
            ),
            //if note 3 is not present
            detector.notePresenceCommandSwitcher(4,
                    //if note 4 is present
                    new SequentialCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-4")),
                        //intake
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom")),
                        //shoot
                        detector.read(),
                        detector.notePresenceCommandSwitcher(5,
                            far5, //if note 5 is present collect it
                            new InstantCommand() //if note 5 is not present do nothing
                        )
                    ),
                    //if note 4 is not present
                    detector.notePresenceCommandSwitcher(5,
                        new SequentialCommandGroup( //if note 5 is present collect it
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-4")),
                            //intake
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom"))
                            //shoot
                        ),
                        new InstantCommand() //if note 5 is not present do nothing
                    )
                )
            )


        );
    }
}