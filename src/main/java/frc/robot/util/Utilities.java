package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;

import static frc.robot.Constants.*;

public class Utilities {
    private Utilities() {} // block instantiation

    /**
     * Returns whether the Driverstation is on the red alliance
     * @return true if on the red alliance, false if on the blue alliance or the alliance is not present
     */
    public static boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isEmpty() || (alliance.get() == DriverStation.Alliance.Red);
    }
    /**
     * Returns where the speaker is on the field
     * @return a {@link Translation2d} that represents where the alliance speaker is on the field
     */

    public static Translation2d getSpeaker() {
        return Utilities.isRedAlliance() ? RED_SPEAKER : BLUE_SPEAKER;
    }

    /**
     * Gets the speaker distance depending on the side the driverstation is on
     * @param robotPos the pos of the robot
     * @return the distance in inches
     */

    public static double getSpeakerDistance(Pose2d robotPos){
        return Math.abs(Units.metersToInches(robotPos.getTranslation().getDistance(Utilities.getSpeaker())));
    }

    /**
     * Gets the robot pos based on the limelight
     * @return a pos2d where the robot is
     */
    public static Pose2d getRobotPos(){
        return LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);
    }
    
    /**
     * Run consumer if object is not null, else do nothing
     * @param <T> type of object
     * @param object input object
     * @param objectConsumer consumer to apply to object
     * @return returns optional input object
     */
    public static <T> Optional<T> runIfNotNull(T object, Consumer<T> objectConsumer) {
        if (object != null) {
            objectConsumer.accept(object);
        }
        return Optional.ofNullable(object);
    }

    public static void removeAndCancelDefaultCommand(Subsystem subsystem) {
        runIfNotNull(subsystem.getDefaultCommand(), (Command command) -> {
            command.cancel();
            subsystem.removeDefaultCommand();
        });
    }

    @SafeVarargs
    public static <T> ArrayList<T> listFromParams(T... items) {
        final ArrayList<T> arrayList = new ArrayList<>();
        for (T item : items) {
            arrayList.add(item);
        }
        return arrayList;
    }

    @SafeVarargs
    public static <T> T[] arrayFromParams(T... items) {
        return items;
    }

    @SafeVarargs
    public static <T> void applyConsumerToParams(Consumer<T> consumer, T... items) {
        for (T item : items) {
            consumer.accept(item);
        }
    }

    public static class WrappedDouble {
        public double val;

        public WrappedDouble(double val) {
            this.val = val;
        }
        
        public WrappedDouble() {
            this(0D);
        }
    }

    public static class WrappedBoolean {
        public boolean val;

        public WrappedBoolean(boolean val) {
            this.val = val;
        }
    }
}
