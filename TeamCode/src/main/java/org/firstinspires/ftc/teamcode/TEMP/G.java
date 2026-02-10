package org.firstinspires.ftc.teamcode.TEMP;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "G")
public class G extends LinearOpMode {
    private DcMotorEx motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "CHANGE NAME LATER");

        waitForStart();
        if (opModeIsActive()) {
            motor.setPower(1);
            while (opModeIsActive()) {
                if (gamepad1.aWasPressed()) motor.setPower(0);
                if (gamepad1.bWasPressed()) motor.setPower(1);
                if (gamepad1.yWasPressed()) motor.setPower(0.5);
            }
        }
    }
}
