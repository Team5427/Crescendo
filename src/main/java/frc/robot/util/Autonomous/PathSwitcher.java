package frc.robot.util.Autonomous;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj2.command.Command;

public interface PathSwitcher {
    public Dictionary<String, Command> pathDict = new Hashtable<>();

    public static void addPath(String pathName, Command pathNumber) {
        pathDict.put(pathName, pathNumber);
    }

    public static Command getPath(String pathName) {
        return pathDict.get(pathName);
    }

    public static void removePath(String pathName) {
        pathDict.remove(pathName);
    }

    public static boolean isEmpty() {
        return pathDict.isEmpty();
    }

    public static int getNumberOfPaths() {
        return pathDict.size();
    }

}
