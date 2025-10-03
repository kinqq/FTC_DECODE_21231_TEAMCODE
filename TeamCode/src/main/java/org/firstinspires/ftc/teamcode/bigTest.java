package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Big Test", group="Linear OpMode")
public class bigTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RevColorSensorV3 color;

    @Override
    public void runOpMode() {
        sub tet = new sub(this);

        color = hardwareMap.get(RevColorSensorV3.class, "bob");

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            tet.magzine();

            telemetry.addData("Runtime: ", runtime);
            //telemetry.addData("Red: ", color.red());
            //telemetry.addData("Green: ", color.green());
            //telemetry.addData("Blue: ", color.blue());
            //telemetry.update();
        }
    }
}