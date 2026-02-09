package org.firstinspires.ftc.teamcode.OLD.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
@Disabled

public class AutomaticMovementTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotorEx.class, "frontRight");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        motor.setTargetPosition(600);
        motor.setPower(1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()
            && motor.isBusy()) {
            //holding the program till timeout or to target position
        }


        // function inchToTick(5) --> 600

//        while (opModeIsActive()) {
//            int ticks = motor.getCurrentPosition();
//
//            telemetry.addData("Motor Position", ticks);
//            telemetry.addData("Motor Revolution", ticks / 537.7);
//
//            telemetry.addData("Motor Distance", ticks / 537.7 * 104 * Math.PI);
//            telemetry.update();
//        }
    }
}
