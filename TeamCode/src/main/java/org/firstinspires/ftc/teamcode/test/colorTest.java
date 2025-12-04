package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Old.colorSensor;


@TeleOp(name="ColorTest", group="Linear OpMode")
//@Disabled
public class colorTest extends LinearOpMode
{
    private RevColorSensorV3 bob = null;
    private RevColorSensorV3 gary = null;
    private RevColorSensorV3 joe = null;

    private Servo servo = null;

    @Override
    public void runOpMode() {
        colorSensor.init(hardwareMap);
        bob = hardwareMap.get(RevColorSensorV3.class, "color");
        gary = hardwareMap.get(RevColorSensorV3.class, "gary");
        joe = hardwareMap.get(RevColorSensorV3.class, "joe");
        servo = hardwareMap.get(Servo.class, "turntable");

        waitForStart();

        double bobP = 0, bobL = 1000, joeP = 0, joeL = 1000, garyP = 0, garyL = 1000;
        double pos = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed())
            {
                pos += 0.4;
            }
            if (gamepad1.rightBumperWasPressed())
            {
                pos -= 0.4;
            }

            if (gamepad1.aWasPressed()) pos += 0.1;
            if (gamepad1.bWasPressed())
            {
                bobP = 0;
                bobL = 1000;
                joeP = 0;
                joeL = 1000;
                garyP = 0;
                garyL = 1000;
            }

            if (pos > 0.8) pos = 0;
            if (pos < 0) pos = 0.8;

            servo.setPosition(pos);


            telemetry.addLine("JOE:");
            telemetry.addData("Joe Detected", colorSensor.colorDetect(colorSensor.joe, 2));
            telemetry.addData("H", colorSensor.H);
            telemetry.addData("S", colorSensor.S);
            telemetry.addData("V", colorSensor.V);
            telemetry.addLine();

            telemetry.addLine("BOB:");
            telemetry.addData("Bob Detected", colorSensor.colorDetect(colorSensor.bob, 0));
            telemetry.addData("H", colorSensor.H);
            telemetry.addData("S", colorSensor.S);
            telemetry.addData("V", colorSensor.V);

            telemetry.addLine("GARY:");
            telemetry.addData("Gary Detected", colorSensor.colorDetect(colorSensor.gary, 1));
            telemetry.addData("H", colorSensor.H);
            telemetry.addData("S", colorSensor.S);
            telemetry.addData("V", colorSensor.V);

            telemetry.addLine();
            telemetry.addData("Distance", bob.getDistance(DistanceUnit.MM));
            if (bob.getDistance(DistanceUnit.MM) > bobP) bobP = bob.getDistance(DistanceUnit.MM);
            if (bob.getDistance(DistanceUnit.MM) < bobL) bobL = bob.getDistance(DistanceUnit.MM);
            telemetry.addData("Bob Peak", bobP);
            telemetry.addData("Bob Low", bobL);

            telemetry.addData("Distance", gary.getDistance(DistanceUnit.MM));
            if (gary.getDistance(DistanceUnit.MM) > garyP) garyP = gary.getDistance(DistanceUnit.MM);
            if (gary.getDistance(DistanceUnit.MM) < garyL) garyL = gary.getDistance(DistanceUnit.MM);
            telemetry.addData("Gary Peak", garyP);
            telemetry.addData("G Low", garyL);

            telemetry.addData("Distance", joe.getDistance(DistanceUnit.MM));
            if (joe.getDistance(DistanceUnit.MM) > joeP) joeP = joe.getDistance(DistanceUnit.MM);
            if (joe.getDistance(DistanceUnit.MM) < joeL) joeL = joe.getDistance(DistanceUnit.MM);
            telemetry.addData("J Peak", joeP);
            telemetry.addData("J Low", joeL);

            telemetry.addLine();
            telemetry.addData("Servo pos", pos);
            telemetry.update();
        }
    }


    private static void waitSeconds(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}