package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;


@TeleOp(name="ColorTest", group="Linear OpMode")
@Disabled
public class colorTest extends LinearOpMode
{
    private RevColorSensorV3 bob = null;
    private RevColorSensorV3 gary = null;
    private RevColorSensorV3 joe = null;

    @Override
    public void runOpMode() {
        bob = hardwareMap.get(RevColorSensorV3.class, "bob");
        gary = hardwareMap.get(RevColorSensorV3.class, "gary");
        joe = hardwareMap.get(RevColorSensorV3.class, "joe");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Bob Red", bob.red());
            telemetry.addData("Bob Green", bob.green());
            telemetry.addData("Bob Blue", bob.blue());

            telemetry.addData("Gary Red", gary.red());
            telemetry.addData("Gary Green", gary.green());
            telemetry.addData("Gary Blue", gary.blue());

            telemetry.addData("Joe Red", joe.red());
            telemetry.addData("Joe Green", joe.green());
            telemetry.addData("Joe Blue", joe.blue());

            telemetry.update();
        }
    }

}