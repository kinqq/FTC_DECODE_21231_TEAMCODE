package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class sub
{

    private final LinearOpMode op;
    private Servo magazing;
    double servp = 0.1;


    private boolean aHeld = false;
    private boolean bHeld = false;

    public sub(LinearOpMode op) {
        this.op = op;
        this.magazing = op.hardwareMap.get(Servo.class, "MG");
        this.magazing.setPosition(servp);
    }

    public void magzine()
    {
        if (op.gamepad1.a && !aHeld) {
            servp += 0.4;
            aHeld = true;
        }
        if (!op.gamepad1.a) {
            aHeld = false;
        }

        if (op.gamepad1.b && !bHeld) {
            servp -= 0.4;
            bHeld = true;
        }
        if (!op.gamepad1.b) {
            bHeld = false;
        }

        if (servp > 0.9) servp = 0.1;
        if (servp < 0.09) servp = 0.9;

        op.telemetry.addData("Servo", servp);
        op.telemetry.update();
        magazing.setPosition(servp);
    }
}