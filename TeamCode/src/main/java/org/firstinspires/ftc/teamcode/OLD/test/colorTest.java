package org.firstinspires.ftc.teamcode.OLD.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OLD.Subsystems.Old.colorSensor;


@TeleOp(name="Color Test", group="Linear OpMode")
@Disabled
public class colorTest extends LinearOpMode
{
    private RevColorSensorV3 bob = null;

    @Override
    public void runOpMode() {
        colorSensor.init(hardwareMap);
        bob = hardwareMap.get(RevColorSensorV3.class, "color");

        double peakD = -1000;
        double lowD = 1000;

        double peakH = -10000;
        double lowH = 10000;

        double peakS = -10000;
        double lowS = 10000;

        double peakV = -10000;
        double lowV = 10000;

        RevColorSensorV3 color = bob;
        String monitoring = "BOB";
        int monitoringIndex = 0;

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.xWasPressed())
            {
                color = bob;
                monitoring = "BOB";
                monitoringIndex = 0;
                peakD = -10000;
                lowD = 10000;
                peakH = -10000;
                lowH = 10000;
                peakS = -10000;
                lowS = 10000;
                peakV = -10000;
                lowV = 10000;
            }

            if (gamepad1.backWasPressed())
            {
                peakD = -1000;
                lowD = 1000;
                peakH = -10000;
                lowH = 10000;
                peakS = -10000;
                lowS = 10000;
                peakV = -10000;
                lowV = 10000;
            }

            telemetry.addData("Monitoring", monitoring);
            telemetry.addData("DETECTED COLOR", colorSensor.colorDetect(color, monitoringIndex));
            telemetry.addData("H", colorSensor.H);
            telemetry.addData(" Peak H", peakH);
            telemetry.addData(" Low H", lowH);
            telemetry.addData("S", colorSensor.S);
            telemetry.addData(" Peak S", peakS);
            telemetry.addData(" Low S", lowS);
            telemetry.addData("V", colorSensor.V);
            telemetry.addData(" Peak V", peakV);
            telemetry.addData(" Low V", lowV);
            telemetry.addData("Dist", color.getDistance(DistanceUnit.MM));
            telemetry.addData(" Peak D", peakD);
            telemetry.addData(" Low D", lowD);
            telemetry.update();

            if (colorSensor.H > peakH) peakH = colorSensor.H;
            if (colorSensor.H < lowH) lowH = colorSensor.H;
            if (colorSensor.S > peakS) peakS = colorSensor.S;
            if (colorSensor.S < lowS) lowS = colorSensor.S;
            if (colorSensor.V > peakV) peakV = colorSensor.V;
            if (colorSensor.V < lowV) lowV = colorSensor.V;
            if (color.getDistance(DistanceUnit.MM) > peakD) peakD = color.getDistance(DistanceUnit.MM);
            if (color.getDistance(DistanceUnit.MM) < lowD) lowD = color.getDistance(DistanceUnit.MM);
        }
    }
}