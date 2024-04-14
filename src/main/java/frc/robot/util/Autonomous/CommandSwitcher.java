package frc.robot.util.Autonomous;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public interface CommandSwitcher {
    public Dictionary<String, SequentialCommandGroup> commandDict = new Hashtable<>();

    public static void addCommand(String pathName, SequentialCommandGroup pathNumber) {
        commandDict.put(pathName, pathNumber);
    }

    public static SequentialCommandGroup getCommand(String pathName) {
        return commandDict.get(pathName);
    }

    public static void removeCommand(String pathName) {
        commandDict.remove(pathName);
    }

    public static boolean isEmpty() {
        return commandDict.isEmpty();
    }

    public static int getNumberOfCommands() {
        return commandDict.size();
    }
}
