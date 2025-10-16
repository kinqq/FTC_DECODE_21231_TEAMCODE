package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class manMag {
    public static Servo magazine;

    public static double servPos;
    public static Servo HA;
    public static DcMotor LA;

    public static void init(HardwareMap hwMap, ElapsedTime run) {
        magazine = hwMap.get(Servo.class, "MG");
        HA = hwMap.get(Servo.class, "HA");
        LA = hwMap.get(DcMotor.class, "LA");
        LA.setDirection(DcMotorSimple.Direction.REVERSE);

        HA.setPosition(0.55);

        double runStart = run.now(TimeUnit.SECONDS);

        while ((run.now(TimeUnit.SECONDS) - runStart) <= 0.5)
        {

        }

        LA.setPower(1);
        servPos = 0;

        magazine.setPosition(servPos);
    }

    public static void moveMag(Gamepad gm1, Gamepad gm2) {
        if (gm2.leftBumperWasPressed()) {
            powerMoters.intakeOff(false);
            if (servPos == 0.6) servPos = 0;
            else if (servPos == 1) servPos = 0.4;
            else if (servPos == 0.2) servPos = 0.8;
            else {
                servPos -= 0.4;
                if (servPos > 0.81) servPos = 0;
                if (servPos < 0) servPos = 0.8;
            }
        }
        if (gm2.rightBumperWasPressed()) {
            powerMoters.intakeOff(false);
            if (servPos == 0.6) servPos = 0;
            else if (servPos == 1) servPos = 0.4;
            else if (servPos == 0.2) servPos = 0.8;
            else {
                servPos += 0.4;
                if (servPos > 0.81) servPos = 0;
                if (servPos < 0) servPos = 0.8;
            }
        }

        if (gm2.yWasPressed()) {
            powerMoters.intakeOff(true);
            if (servPos == 0) servPos = 0.6;
            else if (servPos == 0.4) servPos = 1;
            else if (servPos == 0.8) servPos = 0.2;
        }

        magazine.setPosition(servPos);
    }

    public static void launch(ElapsedTime run) {
        if (servPos == 0.6 || servPos == 1 || servPos == 0.2) {
            HA.setPosition(0.8);

            double runStart = run.now(TimeUnit.SECONDS);

            while ((run.now(TimeUnit.SECONDS) - runStart) <= 0.01) {

            }

            HA.setPosition(0.55);

            runStart = run.now(TimeUnit.SECONDS);

            while ((run.now(TimeUnit.SECONDS) - runStart) <= 0.01) {

            }

            if (servPos == 0.6) servPos = 0;
            else if (servPos == 1) servPos = 0.4;
            else if (servPos == 0.2) servPos = 0.8;
            powerMoters.intakeOff(false);
        }
    }
}
