package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;

public final class AutoCommands {
    //variables naming guide: Ring ID + Starting Location

    public static final double waitTime = 1;

    public static SequentialCommandGroup getFar1ShootTop (Intake intake){ 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top"))
            // shoot todo
    );
}

    public static SequentialCommandGroup getFar2ShootTop (Intake intake){
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top"))
            // shoot todo
    );
}
    
    public static SequentialCommandGroup getFar4ShootBottom (Intake intake){ 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-4")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom"))
             // shoot todo
   );
}

    public static SequentialCommandGroup getFar5ShootBottom (Intake intake){ 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-5")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom"))
            // shoot todo
    );
}

    public static SequentialCommandGroup getFar3ScanTop (Intake intake){
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-3")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Bottom"))
                // shoot todo
    );
}

    public static SequentialCommandGroup getFar3ScanBottom (Intake intake){
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-3")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Top"))
               // shoot todo
    );
}

    public static SequentialCommandGroup getFar1ScanBottom (Intake intake){
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-1")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top"))
               // shoot todo
    );
}

    public static SequentialCommandGroup getFar2ScanBottom (Intake intake){ 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-2")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top"))
             // shoot todo
   );
}

    public static SequentialCommandGroup getFar4ScanTop (Intake intake){ 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-4")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom"))
            // shoot todo
    );
}

    public static SequentialCommandGroup getFar5ScanTop (Intake intake){ 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-5")),
            getIntakeWaiterCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom"))
            // shoot todo
    );
}


    public static Command getIntakeWaiterCommand(Intake intake) {
        return new ParallelRaceGroup(new WaitUntilCommand(() -> intake.notePresent()), new WaitCommand(waitTime));
    }



}
