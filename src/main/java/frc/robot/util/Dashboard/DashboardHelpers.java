package frc.robot.util.Dashboard;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Method to get and put values to SmartDashboard using annotations.
 * <p> To work, the class with the {@link GetVaue} and {@link PutValue} annotations must be registered using the {@link DashboardHelpers#addClassToRefresh(Object)} method. </p>
 * Both {@link GetValue} and {@link PutValue} have 1 parameter which specifies the key to use for the field. If left as an empty string, the field name will be used as the key instead
 * 
 * @author Bradley Chumanov
 */
public class DashboardHelpers {
    private static Map<Object, List<Field>> fieldsToUpdate = new HashMap<>();

    /**
     * Should be called in the constructor of a class.
     * Adds a class to the list of classes to have the fields annotated with {@link GetValue} or {@link PutValue} updated
     * @param clazz the class object. (referenced using "this")
     */
    public static void addClassToRefresh(Object clazz){
        List<Field> fields = Arrays.stream(clazz.getClass().getDeclaredFields()).filter(DashboardHelpers::isUpdateAnnotationPresent).toList();
        for(Field field : fields){
            field.setAccessible(true);
            if(field.isAnnotationPresent(GetValue.class)){
                initGetValueField(clazz, field);
            }
        }
        fieldsToUpdate.put(clazz, fields);
    }

    /**
     * Remove the class from the list to be updated. Should be called in a command's end method. Does not need to be called in a subsystem 
     * @param clazz the class object. (referenced using "this")
     */
    public static void removeClassToRefresh(Object clazz){
        fieldsToUpdate.remove(clazz);
    }

    /**
     * Updates the fields using the annotations provided
     */
    public static void updateValues(){
        for(Entry<Object, List<Field>> classToUpdate : fieldsToUpdate.entrySet()){
            for(Field field : classToUpdate.getValue()){
                for(Annotation annotation : field.getAnnotations()){
                    if(annotation.annotationType().equals(GetValue.class)){
                        updateGetValueField(classToUpdate.getKey(), field);
                    }
                    if(annotation.annotationType().equals(PutValue.class)){
                        updatePutValueField(classToUpdate.getKey(), field);
                    }
                }
            }
        }
    }

    /**
     * Updates a field annotated with {@link GetValue}. Gets the value from SmartDashboard and sets the field.
     * @param clazz the class object
     * @param field the field object
     */
    private static void updateGetValueField(Object clazz, Field field){
        String key = field.getAnnotation(GetValue.class).key();
        if(key.isEmpty()) key = field.getName();
        
        try{
            getDashboardValue(key, field, clazz);
        }catch(IllegalAccessException | IllegalArgumentException e){
            throw new RuntimeException(e);
        }
    }

    private static void initGetValueField(Object clazz, Field field){
        String key = field.getAnnotation(GetValue.class).key();
        if(key.isEmpty()) key = field.getName();

        try{
            putDashboardValue(key, field.get(clazz));
        }catch(IllegalAccessException | IllegalArgumentException e){
            throw new RuntimeException(e);
        }
    }
        
     /**
     * Updates a field annotated with {@link PutValue}. Puts the fields value to SmartDashboard
     * @param clazz the class object
     * @param field the field object
     */
    private static void updatePutValueField(Object clazz, Field field){
        String key = field.getAnnotation(PutValue.class).key();
        if(key.isEmpty()) key = field.getName();

        try{
            putDashboardValue(key, field.get(clazz));
        }catch(IllegalAccessException | IllegalArgumentException e){
            throw new RuntimeException(e);
        }
    }

    /**
     * Puts a type T value to SmartDashboard
     * @param <T> the type of value to put to SmartDashboard
     * @param key the SmartDashboard key
     * @param value the value to put
     */
    private static <T> void putDashboardValue(String key, T value){
        if(value.getClass().equals(boolean.class) || value.getClass().equals(Boolean.class)){
            SmartDashboard.putBoolean(key, (boolean) value);

        }else if(value.getClass().equals(double.class) || value.getClass().equals(Double.class)){
            SmartDashboard.putNumber(key, (double) value);

        }else if(value.getClass().equals(String.class)){
            SmartDashboard.putString(key, (String) value);

        }else{
            throw new IllegalArgumentException(value.getClass().getSimpleName() + " is not a valid puttable SmartDashboard type");
        }
    }

    /**
     * Gets a value from SmartDashboard and assigns it to the field
     * @param <T> the type of value to get
     * @param key the key of the value
     * @param field the field to assign the value to
     * @param clazz the class object
     * @throws IllegalAccessException when the field cannot be accessed
     * @throws IllegalArgumentException when the field is not a valid SmartDashboard type
     */
    private static void getDashboardValue(String key, Field field, Object clazz) throws IllegalAccessException, IllegalArgumentException{
        Class<?> type = field.getType();

        if(type.equals(boolean.class) || type.equals(Boolean.class)){
            field.set(clazz, SmartDashboard.getBoolean(key, (boolean)field.get(clazz)));

        }else if(type.equals(double.class) || type.equals(Double.class)){
            field.set(clazz, SmartDashboard.getNumber(key, (double)field.get(clazz)));

        }else if(type.equals(String.class)){
            field.set(clazz, SmartDashboard.getString(key, (String)field.get(clazz)));

        }else{
            throw new IllegalArgumentException(field.getClass().getSimpleName() + " is not a valid gettable SmartDashboard type");
        }
    }

    private static boolean isUpdateAnnotationPresent(Field field){
        return field.isAnnotationPresent(GetValue.class) || field.isAnnotationPresent(PutValue.class);
    }
}
