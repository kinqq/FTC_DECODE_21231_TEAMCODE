package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="MagazineTest", group="Test")
@Configurable
public class magazineTest extends LinearOpMode
{
    private double servoPos = 0;

    @Override
    public void runOpMode() {
        ServoImplEx mg = hardwareMap.get(ServoImplEx.class, "launchAngle");

        mg.setDirection(Servo.Direction.REVERSE);
        mg.scaleRange(0, .62);

        AnalogInput analog = hardwareMap.get(AnalogInput.class, "encoder");
        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "encoderDigital");

        PanelsConfigurables.INSTANCE.refreshClass(magazineTest.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double voltage = analog.getVoltage();
            double angleDeg = (voltage / 3.3) * 360.0;

            if (gamepad1.aWasPressed()) servoPos += 0.01;
            if (gamepad1.bWasPressed()) servoPos -= 0.01;

            servoPos = Range.clip(servoPos, 0, 1);

            mg.setPosition(servoPos);
            telemetry.addData("Encoder", angleDeg);
            telemetry.addData("Encoder Dig", encoder.getCurrentPosition());
            telemetry.addData("POS", servoPos);
            telemetry.update();

        }
    }

}