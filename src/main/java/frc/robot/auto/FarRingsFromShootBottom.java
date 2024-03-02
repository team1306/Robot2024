package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.NoteDetector;

public class FarRingsFromShootBottom extends SequentialCommandGroup {

    public FarRingsFromShootBottom(NoteDetector detector) {
        addCommands(

            //Collect 4 and 5

            detector.read(),
            detector.notePresenceCommandSwitcher(5,
                //if note 5 present
                new SequentialCommandGroup(
                    Paths.far5ShootBottom, //collect far-5
                    detector.read(),
                    detector.notePresenceCommandSwitcher(4,
                        Paths.far4ShootBottom, //if note 4 is present collect it
                        new InstantCommand() //if note 4 is not present do nothing
                    )
                ),
                //if note 5 not present
                detector.notePresenceCommandSwitcher(4,
                    Paths.far4ShootBottom, //if note 4 is present collect it
                    //if note 2 not present
                    new InstantCommand() //if note 4 is not present do nothing
                )
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Scan-Bottom")), //Go to scan for the top rings

            //Collect 1, 2, and 3

            detector.read(),
            detector.notePresenceCommandSwitcher(3,
            //if note 3 is present
            new SequentialCommandGroup( //get note 3 and go to Shoot-Top
                Paths.far3ScanBottom,
                detector.read(),
                detector.notePresenceCommandSwitcher(2,
                    //if note 2 is present
                    new SequentialCommandGroup(
                        Paths.far2ShootTop,
                        detector.read(),
                        detector.notePresenceCommandSwitcher(1,
                            Paths.far1ShootTop, //if note 1 is present collect it
                            new InstantCommand() //if note 1 is not present do nothing
                        )
                    ),
                    //if note 2 is not present
                    detector.notePresenceCommandSwitcher(1,
                        Paths.far1ShootTop, //if note 1 is present collect it
                        new InstantCommand() //if note 1 is not present do nothing
                    )
                )
            ),
            //if note 3 is not present
            detector.notePresenceCommandSwitcher(2,
                    //if note 2 is present
                    new SequentialCommandGroup(
                        Paths.far2ScanBottom,
                        detector.read(),
                        detector.notePresenceCommandSwitcher(1,
                            Paths.far1ShootTop, //if note 1 is present collect it
                            new InstantCommand() //if note 1 is not present do nothing
                        )
                    ),
                    //if note 2 is not present
                    detector.notePresenceCommandSwitcher(1,
                        Paths.far1ScanBottom,
                        new InstantCommand() //if note 1 is not present do nothing
                    )
                )
            )


        );
    }
}