package frc.robot.utils;
import static edu.wpi.first.units.Units.Newton;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.FileSystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

public class dave_preferences {

    public static class json_content {
        public HashMap<String, Double> storedValues = new HashMap<String, Double>();
    }


    private static json_content prefs = new json_content();
    private static File directory = Filesystem.getDeployDirectory();
    private static String filepath = directory + "/arbitrary_values.json";

    static {
        String content = null;
        try {
            Scanner sc = new Scanner(new File(filepath)).useDelimiter("\\Z");
            content = sc.next();
            sc.close();
            System.out.println(content);
            var object_mapper = new ObjectMapper();
            try {
                prefs = object_mapper.readValue(content, json_content.class);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static double get_saved_value(String key) {
        if (prefs.storedValues.containsKey(key))
        {
            System.out.println("Value found");
            return prefs.storedValues.get(key);
        }
        else
        {
            System.out.println("VALUE NOT FOUND");
            return 0.0;
        }
    }

    public static void store_or_change_value(String key, double value)
    {
        if (prefs.storedValues.containsKey(key))
        {
            System.out.println("Value stored at existing key");
            prefs.storedValues.replace(key, value);
        }
        else
        {
            System.out.println("NEW key and value stored");
            prefs.storedValues.put(key, value);
        }
    }

    public static void delete_key_value_pair(String key)
    {
        if (prefs.storedValues.containsKey(key))
        {
            System.out.println("Key/Value Pair Deleted");
            prefs.storedValues.remove(key);
        }
        else
        {
            System.out.println("KEY NOT FOUND");
        }
    }

    public static void save_file()
    {
        var file = new File(filepath);
        try {
            var fw = new FileWriter(file);
            String content = new ObjectMapper().writeValueAsString(prefs);
            fw.write(content);
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}


    /*
    public static class json_content {
        public ArrayList<String> keys = new ArrayList<>();
        public ArrayList<Double> locks = new ArrayList<>();
    }

    private static json_content prefs = new json_content();
    private static File directory = Filesystem.getDeployDirectory();
    private static String filepath = directory + "/arbitrary_values.json";

    public static void print_values() {
        var directory = Filesystem.getDeployDirectory();
        var filepath = directory + "/arbitrary_values.json";
        String content;
        try {
            Scanner sc = new Scanner(new File(filepath)).useDelimiter("\\Z");
            content = sc.next();
            sc.close();
            System.out.println(content);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return;
        }

        var object_mapper = new ObjectMapper();
        try {
            var data = object_mapper.readValue(content, json_content.class);
            System.out.println(data);
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
    }

    static {
        String content = null;
        try {
            Scanner sc = new Scanner(new File(filepath)).useDelimiter("\\Z");
            content = sc.next();
            sc.close();
            System.out.println(content);
            var object_mapper = new ObjectMapper();
            try {
                prefs = object_mapper.readValue(content, json_content.class);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static double get_saved_value(String key) {
        for (int i = 0; i < prefs.keys.size(); i++)
        {
            if (prefs.keys.get(i) == key)
            {
                return prefs.locks.get(i);
            }
        }
        System.out.println("VALUE NOT FOUND");
        return 0.0;
    }

    public static void save_value(String key, double value, boolean write_to_file) {
        boolean valueStored = false;
        for (int i = 0; i < prefs.keys.size(); i++)
        {
            if (prefs.keys.get(i) == key)
            {
                System.out.println("Value Updated");
                prefs.locks.set(i, value);
                valueStored = true;
            }
        }
        
        if (!valueStored) {
            System.out.println("New Value Stored");
            prefs.keys.add(key);
            prefs.locks.add(value);
        }

        if(write_to_file) {
            var file = new File(filepath);
            try {
                var fw = new FileWriter(file);
                String content = new ObjectMapper().writeValueAsString(prefs);
                fw.write(content);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    } */

