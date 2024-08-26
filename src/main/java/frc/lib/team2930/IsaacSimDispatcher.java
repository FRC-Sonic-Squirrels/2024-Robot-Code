// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930;

import java.util.ArrayList;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class IsaacSimDispatcher {
    
    private NetworkTableInstance inst;

    private NetworkTable table;

    private ArrayList<DoublePublisher> publishers;
    private ArrayList<DoubleSubscriber> subscribers;

    private int[] motorIDs;
    private String[] sensorNames;

    public IsaacSimDispatcher(int[] motorIDs, String[] sensorNames){
        this.motorIDs = motorIDs;
        this.sensorNames = sensorNames;
        inst = NetworkTableInstance.getDefault();
        inst.setServerTeam(2930);
        inst.startClient4("IsaacSimDispatcher");
        inst.startDSClient();
        table = inst.getTable("Sonic Sim");
        publishers = new ArrayList<>(motorIDs.length);
        subscribers = new ArrayList<>(sensorNames.length);
        for (int motorID : motorIDs) {
            publishers.add(table.getDoubleTopic("motor" + motorID).publish());
        }

        for (String sensorName : sensorNames) {
            subscribers.add(table.getDoubleTopic(sensorName).subscribe(0));
        }
    }

    public void sendMotorInfo(int canID, double value){
        publishers.get(indexOfIntArray(motorIDs, canID)).set(value);
    }

    public double recieveSensorInfo(String ID){
        return subscribers.get(indexOfStringArray(sensorNames, ID)).get();
    }

    private int indexOfIntArray(int[] array, int integer){
        int index = 0;
        for (int i : array) {
            if(i==integer){
                return index;
            }
            index++;
        }
        return -1;
    }

    private int indexOfStringArray(String[] array, String string){
        int index = 0;
        for (String str : array) {
            if(str==string){
                return index;
            }
            index++;
        }
        return -1;
    }
}
