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
    private ArrayList<DoubleSubscriber> motorPosSubscribers;
    private ArrayList<DoubleSubscriber> motorVelSubscribers;
    private ArrayList<DoubleSubscriber> sensorSubscribers;

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
        motorPosSubscribers = new ArrayList<>(motorIDs.length);
        motorVelSubscribers = new ArrayList<>(motorIDs.length);
        sensorSubscribers = new ArrayList<>(sensorNames.length);
        for (int motorID : motorIDs) {
            publishers.add(table.getDoubleTopic("motor" + motorID + "/input").publish());
            motorPosSubscribers.add(table.getDoubleTopic("motor" + motorID + "/output/pos").subscribe(0));
            motorVelSubscribers.add(table.getDoubleTopic("motor" + motorID + "/output/vel").subscribe(0));
        }

        for (String sensorName : sensorNames) {
            sensorSubscribers.add(table.getDoubleTopic(sensorName).subscribe(0));
        }
    }

    public void sendMotorInfo(int canID, double value){
        publishers.get(indexOfIntArray(motorIDs, canID)).set(value);
        inst.flushLocal();
        inst.flush();
    }

    public double recieveMotorPos(int canID){
        return motorPosSubscribers.get(indexOfIntArray(motorIDs, canID)).get();
    }

    public double recieveMotorVel(int canID){
        return motorVelSubscribers.get(indexOfIntArray(motorIDs, canID)).get();
    }

    public double recieveSensorInfo(String ID){
        return sensorSubscribers.get(indexOfStringArray(sensorNames, ID)).get();
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
