package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Hood Test", group="Test")
@Configurable

public class HoodAngleTest extends LinearOpMode
{
    private double servoPos = 0;

    @Override
    public void runOpMode() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "launchAngle");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servo.setDirection(Servo.Direction.REVERSE);

        PanelsConfigurables.INSTANCE.refreshClass(HoodAngleTest.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) servoPos += 0.1;
            if (gamepad1.bWasPressed()) servoPos -= 0.1;
            if (gamepad1.yWasPressed()) servoPos += 0.01;
            if (gamepad1.xWasPressed()) servoPos -= 0.01;

            servoPos = Range.clip(servoPos, 0.19, 0.85);
            servo.setPosition(servoPos);

            telemetry.addData("POS", servoPos);
            telemetry.addData("REAL POS", servo.getPosition());
            telemetry.update();

        }
    }

}