package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Subsystems.Old.colorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Old.magazine;
import org.firstinspires.ftc.teamcode.Subsystems.Old.powerMotors;
import org.firstinspires.ftc.teamcode.Subsystems.Old.turret;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.Constants;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Not so Basic Drive")
@Configurable
public class testTeleop extends OpMode
{
    boolean g = true;
    @Override
    public void init() {}

    public void loop() {while (g) {}

    }

    @Override
    public void stop() {g = false;}
}