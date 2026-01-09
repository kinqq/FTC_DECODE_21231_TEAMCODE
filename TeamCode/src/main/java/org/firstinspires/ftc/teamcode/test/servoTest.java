package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Servo Test", group="Test")
@Configurable
public class servoTest extends LinearOpMode
{
    private double servoPos = 0;

    @Override
    public void runOpMode() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "launchAngle");
        //DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "leftFront");

        //servo.scaleRange(0.189, 0.5);
        servo.setDirection(Servo.Direction.REVERSE);

        PanelsConfigurables.INSTANCE.refreshClass(servoTest.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) servoPos += 0.1;
            if (gamepad1.bWasPressed()) servoPos -= 0.1;
            if (gamepad1.yWasPressed()) servoPos += 0.01;
            if (gamepad1.xWasPressed()) servoPos -= 0.01;

            servoPos = Range.clip(servoPos, 0, 1);
            servo.setPosition(servoPos);

            telemetry.addData("POS", servoPos);
            telemetry.addData("REAL POS", servo.getPosition());
            telemetry.update();

        }
    }

}