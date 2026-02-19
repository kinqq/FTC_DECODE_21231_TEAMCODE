package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Angle Test", group = "Test")
public class TurretAngleTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        double deg = 0;

        waitForStart();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.bWasPressed()) deg += 5;
            if (gamepad1.aWasPressed()) deg -= 5;

            deg = Range.clip(deg, -185, 185);
            int targetReal = (int) Math.round((deg * 5.6111111111) * 384.5 / 360.0);

            motor.setTargetPosition(targetReal);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);

            telemetry.addData("Position", deg);
            telemetry.update();
        }


    }

}
