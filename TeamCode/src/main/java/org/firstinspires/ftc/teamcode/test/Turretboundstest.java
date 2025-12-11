package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@TeleOp(name = "Test Turret Bounds")
public class Turretboundstest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        double deg = 0;

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            if (gamepad1.bWasPressed()) deg += 5;
            if (gamepad1.aWasPressed()) deg -= 5;

            deg = Range.clip(deg, -115, 215);
            double target = deg;
            ;//Range.clip(deg - 10, -235, 85);
            double dTarget = target * 5.6111111111;
            int fTarget = (int) Math.round(dTarget * 537.7 / 360.0);


            motor.setTargetPosition(fTarget);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);

            telemetry.addData("Position", deg);
            telemetry.update();
        }


    }

}
