// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** 
 * Integer that is debuggble through Shuffleboard
 * ONLY USE IN DEBUG MODE
 */
public class DebugNum {
    public static ArrayList<DebugNum> debugNums = new ArrayList<DebugNum>();
    private double value;
    private double initValue;
    private String name;

    public DebugNum(String name, int value) {
        this.name = name;
        this.value = value;
        this.initValue = value;
        debugNums.add(this);
    }

    public void set() {
        NetworkTableEntry entry = NetworkTableInstance.getDefault().getEntry(name);
        value = entry.getDouble(initValue);

    }

    public double get() {
        NetworkTableEntry entry = NetworkTableInstance.getDefault().getEntry(name);
        return entry.getDouble(initValue);
    }

    public String getName() {
        return name;
    }

    public void log() {
        Shuffleboard.getTab("debug").add(name, value);
    }
}
