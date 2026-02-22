package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BreakBeamCommands {
    private DigitalChannel beam1, beam2;

    public BreakBeamCommands(HardwareMap hwMap) {
        beam1 = hwMap.get(DigitalChannel.class, "digital1");
        beam2 = hwMap.get(DigitalChannel.class, "digital2");
    }

    public boolean isArtifactPresent() {
        return !(beam1.getState() && beam2.getState());
    }

    public boolean beam1State() {
        return beam1.getState();
    }

    public boolean beam2State() {
        return beam2.getState();
    }
}
