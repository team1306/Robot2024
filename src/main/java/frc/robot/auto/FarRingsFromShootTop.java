package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.subsystems.vision.NoteDetector.Checkpoints;

public class FarRingsFromShootTop extends SequentialCommandGroup {

    public FarRingsFromShootTop(NoteDetector detector, Intake intake, Arm arm, Shooter shooter) {
        addCommands(

            //Collect 1 and 2
            detector.read(Checkpoints.TOP),
            detector.notePresenceCommandSwitcher(1,
                //if note 1 present
                new SequentialCommandGroup(
                    AutoCommands.getFar1ShootTop(intake), //collect far-1
                    detector.read(Checkpoints.TOP),
                    detector.notePresenceCommandSwitcher(2,
                        AutoCommands.getFar2ShootTop(intake), //if note 2 is present collect it
                        new InstantCommand() //if note 2 is not present do nothing
                    )
                ),
                //if note 1 not present
                detector.notePresenceCommandSwitcher(2,
                    AutoCommands.getFar2ShootTop(intake), //if note 2 is present collect it
                    //if note 2 not present
                    new InstantCommand() //if note 2 is not present do nothing
                )
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Scan-Top")), //Go to scan for the bottom rings

            //Collect 3, 4, and 5

            detector.read(Checkpoints.BOTTOM_MIDDLE), // fix this checkpoint
            detector.notePresenceCommandSwitcher(3,
            //if note 3 is present
            new SequentialCommandGroup( //get note 3 and go to Shoot-Bottom
                AutoCommands.getFar3ScanTop(intake),
                detector.read(Checkpoints.TOP), // fix this checkpoint
                detector.notePresenceCommandSwitcher(4,
                    //if note 4 is present
                    new SequentialCommandGroup(
                        AutoCommands.getFar4ShootBottom(intake),
                        detector.read(Checkpoints.TOP), // fix this checkpoint
                        detector.notePresenceCommandSwitcher(5,
                            AutoCommands.getFar5ShootBottom(intake), //if note 5 is present collect it
                            new InstantCommand() //if note 5 is not present do nothing
                        )
                    ),
                    //if note 4 is not present
                    detector.notePresenceCommandSwitcher(5,
                        AutoCommands.getFar5ShootBottom(intake), //if note 5 is present collect it
                        new InstantCommand() //if note 5 is not present do nothing
                    )
                )
            ),
            //if note 3 is not present
            detector.notePresenceCommandSwitcher(4,
                    //if note 4 is present
                    new SequentialCommandGroup(
                        AutoCommands.getFar4ScanTop(intake),
                        detector.read(Checkpoints.BOTTOM_MIDDLE), // fix this checkpoint
                        detector.notePresenceCommandSwitcher(5,
                            AutoCommands.getFar5ShootBottom(intake), //if note 5 is present collect it
                            new InstantCommand() //if note 5 is not present do nothing
                        )
                    ),
                    //if note 4 is not present
                    detector.notePresenceCommandSwitcher(5,
                        AutoCommands.getFar5ScanTop(intake),
                        new InstantCommand() //if note 5 is not present do nothing
                    )
                )
            )


        );
    }
}