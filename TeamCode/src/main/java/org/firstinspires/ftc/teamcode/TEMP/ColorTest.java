package org.firstinspires.ftc.teamcode.TEMP;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TEMP.Commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.TEMP.Constants.Enums.ArtifactColors;

@TeleOp(name = "ColorTest")public class ColorTest extends LinearOpMode {
    private NormalizedColorSensor colorSensor;
    private DcMotorEx intake;
    private Servo g;

    private IndexerCommands indCmds;


    @Override public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        indCmds = new IndexerCommands(hardwareMap);
        g = hardwareMap.get(Servo.class, "hammer");
        g.setPosition(0.65);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(1);

        waitForStart();

        indCmds.start();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                indCmds.update();
                if (!indCmds.indexerIsMoving()) indCmds.getDetectedColor();

                if (indCmds.getIntakeSlotColor() != ArtifactColors.UNKNOWN) indCmds.nextSlot();
            }
        }
    }
}
