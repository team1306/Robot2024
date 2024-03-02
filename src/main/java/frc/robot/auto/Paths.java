package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Paths {
    
    public static final SequentialCommandGroup far1ShootTop = new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top"))
            //shoot
    );

    public static final SequentialCommandGroup far2ShootTop = new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top"))
            //shoot
    );
    
    public static final SequentialCommandGroup far4ShootBottom = new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-4")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom"))
            //shoot
    );

    public static final SequentialCommandGroup far5ShootBottom = new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-5")),
            //intake
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom"))
            //shoot
    );

}
