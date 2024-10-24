package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class FarRingsFromStartBottom extends ParallelCommandGroup {

    public FarRingsFromStartBottom(NoteDetector detector, Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {

        System.out.println("Running Auto");
        /*
            addCommands(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-3 to Shoot-Bottom")), //go to shoot bottom
            //Collect 4 and 5
            detector.read(Checkpoints.BOTTOM_MIDDLE), // FIX THIS CHECKPOINT
            detector.notePresenceCommandSwitcher(5,
                //if note 5 present
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        AutoCommands.getFar5ShootBottom(intake, shooter, arm, driveTrain)) , //collect far-5
                    detector.read(Checkpoints.BOTTOM_MIDDLE), // FIX THIS CHECKPOINT
                    detector.notePresenceCommandSwitcher(4,
                        AutoCommands.getFar4ShootBottom(intake, shooter, arm, driveTrain), //if note 4 is present collect it
                        new InstantCommand() //if note 4 is not present do nothing
                    )
                ),
                //if note 5 not present
                detector.notePresenceCommandSwitcher(4,
                    AutoCommands.getFar4ShootBottom(intake, shooter, arm, driveTrain), //if note 4 is present collect it
                    //if note 2 not present
                    new InstantCommand() //if note 4 is not present do nothing
                )
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Scan-Bottom")), //Go to scan for the top rings

            //Collect 1, 2, and 3

            detector.read(Checkpoints.BOTTOM_MIDDLE), // FIX THIS CHECKPOINT
            detector.notePresenceCommandSwitcher(3,
            //if note 3 is present
            new SequentialCommandGroup( //get note 3 and go to Shoot-Top
                AutoCommands.getFar3ScanBottom(intake, shooter, arm, driveTrain),
                detector.read(Checkpoints.BOTTOM_MIDDLE),// FIX THIS CHECKPOINT
                detector.notePresenceCommandSwitcher(2,
                    //if note 2 is present
                    new SequentialCommandGroup(
                        AutoCommands.getFar2ShootTop(intake, shooter, arm, driveTrain),
                        detector.read(Checkpoints.BOTTOM_MIDDLE), // FIX THIS CHECKPOINT
                        detector.notePresenceCommandSwitcher(1,
                            AutoCommands.getFar1ShootTop(intake, shooter, arm, driveTrain), //if note 1 is present collect it
                            new InstantCommand() //if note 1 is not present do nothing
                        )
                    ),
                    //if note 2 is not present
                    detector.notePresenceCommandSwitcher(1,
                        AutoCommands.getFar1ShootTop(intake, shooter, arm, driveTrain), //if note 1 is present collect it
                        new InstantCommand() //if note 1 is not present do nothing
                    )
                )
            ),
            //if note 3 is not present
            detector.notePresenceCommandSwitcher(2,
                    //if note 2 is present
                    new SequentialCommandGroup(
                        AutoCommands.getFar2ScanBottom(intake, shooter, arm, driveTrain),
                        detector.read(Checkpoints.BOTTOM_MIDDLE), // FIX THIS CHECKPOINT
                        detector.notePresenceCommandSwitcher(1,
                            AutoCommands.getFar1ShootTop(intake, shooter, arm, driveTrain), //if note 1 is present collect it
                            new InstantCommand() //if note 1 is not present do nothing
                        )
                    ),
                    //if note 2 is not present
                    detector.notePresenceCommandSwitcher(1,
                        AutoCommands.getFar1ScanBottom(intake, shooter, arm, driveTrain),
                        new InstantCommand() //if note 1 is not present do nothing
                    )
                )
            )


        );
        */
    }
    
    @Override
    public String getName() {
        return "Far Rings from Start Bottom";
    }
}