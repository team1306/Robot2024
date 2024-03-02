package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.NoteDetector;

public class FarRingsFromShootTop extends SequentialCommandGroup {

    public FarRingsFromShootTop(NoteDetector detector) {
        addCommands(

            //Collect 1 and 2

            detector.read(),
            detector.notePresenceCommandSwitcher(1,
                //if note 1 present
                new SequentialCommandGroup(
                    Paths.far1ShootTop, //collect far-1
                    detector.read(),
                    detector.notePresenceCommandSwitcher(2,
                        Paths.far2ShootTop, //if note 2 is present collect it
                        new InstantCommand() //if note 2 is not present do nothing
                    )
                ),
                //if note 1 not present
                detector.notePresenceCommandSwitcher(2,
                    Paths.far2ShootTop, //if note 2 is present collect it
                    //if note 2 not present
                    new InstantCommand() //if note 2 is not present do nothing
                )
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Scan-Top")), //Go to scan for the bottom rings

            //Collect 3, 4, and 5

            detector.read(),
            detector.notePresenceCommandSwitcher(3,
            //if note 3 is present
            new SequentialCommandGroup( //get note 3 and go to Shoot-Bottom
                Paths.far3ScanTop,
                detector.read(),
                detector.notePresenceCommandSwitcher(4,
                    //if note 4 is present
                    new SequentialCommandGroup(
                        Paths.far4ShootBottom,
                        detector.read(),
                        detector.notePresenceCommandSwitcher(5,
                            Paths.far5ShootBottom, //if note 5 is present collect it
                            new InstantCommand() //if note 5 is not present do nothing
                        )
                    ),
                    //if note 4 is not present
                    detector.notePresenceCommandSwitcher(5,
                        Paths.far5ShootBottom, //if note 5 is present collect it
                        new InstantCommand() //if note 5 is not present do nothing
                    )
                )
            ),
            //if note 3 is not present
            detector.notePresenceCommandSwitcher(4,
                    //if note 4 is present
                    new SequentialCommandGroup(
                        Paths.far4ScanTop,
                        detector.read(),
                        detector.notePresenceCommandSwitcher(5,
                            Paths.far5ShootBottom, //if note 5 is present collect it
                            new InstantCommand() //if note 5 is not present do nothing
                        )
                    ),
                    //if note 4 is not present
                    detector.notePresenceCommandSwitcher(5,
                        Paths.far5ScanTop,
                        new InstantCommand() //if note 5 is not present do nothing
                    )
                )
            )


        );
    }
}