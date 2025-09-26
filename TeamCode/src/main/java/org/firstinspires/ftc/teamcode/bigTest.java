package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Big Test", group="Linear OpMode")
public class bigTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo bob = null;


    @Override
    public void runOpMode() {
        bob = hardwareMap.get(Servo.class, "bob");

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            bob.setPosition(60);

            telemetry.addData("runtime:", runtime);
            telemetry.update();
        }
    }
}
