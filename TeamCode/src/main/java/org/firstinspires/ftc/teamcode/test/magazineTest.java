package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="MagazineTest", group="Test")
@Disabled
public class magazineTest extends LinearOpMode
{
    private double servoPos = 0.5;

    @Override
    public void runOpMode() {
        Servo mg = hardwareMap.get(Servo.class, "MG");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.rightBumperWasPressed()) servoPos += 0.1;
            if(gamepad1.leftBumperWasPressed()) servoPos -= 0.1;
            if(gamepad1.aWasPressed()) servoPos += 0.01;
            if(gamepad1.bWasPressed()) servoPos -= 0.01;

            mg.setPosition(servoPos);

            telemetry.addData("Servo Position: ", mg.getPosition());
            telemetry.update();
        }
    }

}