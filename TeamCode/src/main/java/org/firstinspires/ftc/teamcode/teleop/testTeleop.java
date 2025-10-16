package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Subsystems.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "VERYBasicDrive")
@Configurable
public class testTeleop extends OpMode {

    public static ElapsedTime runtime = new ElapsedTime();
    public float timer = 2;

    public boolean pressed1 = false;
    public boolean pressed2 = false;

    @Override
    public void init()
    {
        runtime.reset();

        powerMoters.init(hardwareMap); //Initialize the drive subsystem
        magazine.init(hardwareMap); //Initialize magazine subsystem
        //manMag.init(hardwareMap, runtime);
        colorSensor.init(hardwareMap);

        powerMoters.intakeOff(false);

    }

    @Override
    public void loop() {
        //Send stick values to drive for odo drive
        powerMoters.basicDrive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        //Control intake manually
        if (gamepad2.a) powerMoters.intakeOff(false);
        if (gamepad2.b) powerMoters.intakeOff(true);

        //Wait to check till 0.5 seconds pass
        if (runtime.now(TimeUnit.SECONDS) - timer >= 2) {
            magazine.checkColors(); //Check and update all magazine slots
            timer = runtime.now(TimeUnit.SECONDS); //Reset timer to now to reflect new countdown
        }

        if (gamepad2.leftBumperWasPressed()) magazine.find(0);//Find a purple and put in launch position
        if (gamepad2.rightBumperWasPressed()) magazine.find(1); //Find a green and put in launch position


        telemetry.addData("Detected Color", colorSensor.colorDetect(colorSensor.bob));
        //manMag.moveMag(gamepad1, gamepad2);

        //if (gamepad2.aWasPressed()) manMag.launch(runtime);

        //telemetry.addData("Servo Position", manMag.magazine.getPosition());
        //telemetry.addData("Servo Position (Reported)", manMag.servPos * 10);

    }
}
