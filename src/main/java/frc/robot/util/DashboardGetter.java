package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Class to simplify the getting and assigning of variables from {@link SmartDashboard}
 */
@Deprecated
public class DashboardGetter {
    private static final List<GetEntry> getEntries = new ArrayList<>();

    private interface GetEntry{
        /**
         * Method for all implementing classes to call their respective {@link Consumer} with values from {@link SmartDashboard}
         */
        void updateVariable();
    }
    private static abstract class GetData<T> {
        public final String key;
        public final T defaultValue;
        public final Consumer<T> consumer;
        private GetData(String key, T defaultValue, Consumer<T> consumer){

            this.key = key;
            this.defaultValue = defaultValue;
            this.consumer = consumer;
        }
    }

    private static class GetDoubleData extends GetData<Double> implements GetEntry{

        private GetDoubleData(String key, Double defaultValue, Consumer<Double> consumer) {
            super(key, defaultValue, consumer);
        }

        @Override
        public void updateVariable() {
            consumer.accept(SmartDashboard.getNumber(key, defaultValue));
        }
    }

    private static class GetBooleanData extends GetData<Boolean> implements GetEntry{

        private GetBooleanData(String key, Boolean defaultValue, Consumer<Boolean> consumer) {
            super(key, defaultValue, consumer);
        }

        @Override
        public void updateVariable() {
            consumer.accept(SmartDashboard.getBoolean(key, defaultValue));
        }
    }

    private static class GetSendableData extends GetData<Sendable> implements GetEntry{

        private GetSendableData(String key, Consumer<Sendable> consumer) {
            super(key, null, consumer);
        }

        @Override
        public void updateVariable() {
            consumer.accept(SmartDashboard.getData(key));
        }
    }

    /**
     * Updates all the variables given in the {@link Consumer} of each {@link GetData GetData}
     * Should only be called once in {@link Robot#robotPeriodic()}
     */
    public static void update(){
        for(GetEntry entry : getEntries){
            entry.updateVariable();
        }
    }

    /**
     * Add an entry of a {@link GetDoubleData} to the entries to be updated
     * {@link Consumer Consumers} should be structured as follows: value -> valueToBeUpdated = value
     * valueToBeUpdated should be replaced with the actual value to be updated and needs to be a global variable
     * @param key the key to use to get data from {@link SmartDashboard}
     * @param defaultValue the default value to use if the key in {@link SmartDashboard} does not exist
     * @param consumer the {@link Consumer} to update a variable. See structure of it above
     */
    public static void addGetDoubleData(String key, double defaultValue, Consumer<Double> consumer){
        getEntries.add(new GetDoubleData(key, defaultValue, consumer));
        SmartDashboard.putNumber(key, defaultValue);
    }

    /**
     * Add an entry of a {@link GetBooleanData} to the entries to be updated
     * @see #addGetDoubleData(String, double, Consumer)
     */
    public static void addGetBooleanData(String key, boolean defaultValue, Consumer<Boolean> consumer){
        getEntries.add(new GetBooleanData(key, defaultValue, consumer));
        SmartDashboard.putBoolean(key, defaultValue);
    }

    /**
     * Add an entry of a {@link GetSendableData} to the entries to be updated
     * @see #addGetDoubleData(String, double, Consumer)
     */
    public static void addGetSendableData(String key, Consumer<Sendable> consumer){
        getEntries.add(new GetSendableData(key, consumer));
        SmartDashboard.putData(key, null);
    }
}
