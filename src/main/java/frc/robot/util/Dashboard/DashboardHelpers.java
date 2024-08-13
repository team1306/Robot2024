package frc.robot.util.Dashboard;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardHelpers {
    private static Map<Object, List<Field>> fieldsToUpdate = new HashMap<>();

    /**
     * Should be called in the constructor of a class.
     * Adds a class to the list of classes to have the fields annotated with {@link GetValue} or {@link PutValue} updated
     * @param clazz the class object. (referenced using "this")
     */
    public static void addClassToRefresh(Object clazz){
        List<Field> fields = Arrays.stream(clazz.getClass().getDeclaredFields()).filter(DashboardHelpers::isUpdateAnnotationPresent).toList();
        fieldsToUpdate.put(clazz, fields);
    }

    /**
     * Remove the class from the list to be updated. Should be called in a command's end method. Does not need to be called in a subsystem 
     * @param clazz the class object. (referenced using "this")
     */
    public static void removeClassToRefresh(Object clazz){
        fieldsToUpdate.remove(clazz);
    }

    public static void updateValues(){
        for(Entry<Object, List<Field>> classToUpdate : fieldsToUpdate.entrySet()){
            for(Field field : classToUpdate.getValue()){
                for(Annotation annotation : field.getAnnotations()){
                    if(annotation.annotationType().equals(GetValue.class)){
                        field.setAccessible(true);
                        updateGetValueField(classToUpdate.getKey(), field);
                    }
                    if(annotation.annotationType().equals(PutValue.class)){
                        field.setAccessible(true);
                        updatePutValueField(classToUpdate.getKey(), field);
                    }
                }
                // if(field.isAnnotationPresent(PutValue.class)){
                //     field.setAccessible(true);
                //     updatePutValueField(classToUpdate.getKey(), field);
                // }
            }
        }
    }

    private static void updateGetValueField(Object clazz, Field field){
        Class<?> type = field.getType();
        String key = field.getAnnotation(GetValue.class).key();
        try{
            if(type.equals(boolean.class)){
                field.set(clazz, SmartDashboard.getBoolean(key, (boolean)field.get(clazz)));

            }else if(type.equals(double.class)){
                field.set(clazz, SmartDashboard.getNumber(key, (double)field.get(clazz)));

            }else if(type.equals(String.class)){
                field.set(clazz, SmartDashboard.getString(key, (String)field.get(clazz)));

            }else{
                throw new IllegalArgumentException(field.getName() + " is not a valid SmartDashboard type");
            }
        }catch(IllegalAccessException | IllegalArgumentException e){
            throw new RuntimeException(e);
        }
    }
        
    private static void updatePutValueField(Object clazz, Field field){
        Class<?> type = field.getType();
        String key = field.getAnnotation(GetValue.class).key();
         try{
            if(type.equals(boolean.class)){
                SmartDashboard.putBoolean(key, (boolean) field.get(clazz));

            }else if(type.equals(double.class)){
                SmartDashboard.putNumber(key, (double) field.get(clazz));

            }else if(type.equals(String.class)){
                SmartDashboard.putString(key, (String) field.get(clazz));

            }else{
                throw new IllegalArgumentException(field.getName() + " is not a valid SmartDashboard type");
            }
        }catch(IllegalAccessException | IllegalArgumentException e){
            throw new RuntimeException(e);
        }
    }

    private static boolean isUpdateAnnotationPresent(Field field){
        return field.isAnnotationPresent(GetValue.class) || field.isAnnotationPresent(PutValue.class);
    }
}
