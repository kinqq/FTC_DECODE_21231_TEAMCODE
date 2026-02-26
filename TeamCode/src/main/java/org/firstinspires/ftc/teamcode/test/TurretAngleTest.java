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

        double deg = 0;
        boolean run = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) deg += 5;
            if (gamepad1.leftBumperWasPressed()) deg -= 5;

            deg = Range.clip(deg, -175, 175);
            int targetReal = (int) Math.round((deg * 5.6111111111) * 384.5 / 360.0);

            if (gamepad1.aWasPressed()) run = true;
            if (gamepad1.bWasPressed()) run = false;

            if (gamepad1.startWasPressed())
            {
                run = false;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (run)
            {
                motor.setTargetPosition(targetReal);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            else motor.setPower(0);

            telemetry.addData("Position", deg);
            telemetry.addData("Ticks", motor.getCurrentPosition());
            telemetry.update();
        }


    }

}
