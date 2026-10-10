package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.speeds;

@Disabled
//@TeleOp(name="Short Circuits: Flywheel Test", group="Linear OpMode")
public class FlywheelTestOpMode extends OpMode {

    private DcMotorEx launcher = null;
    private CRServo windMill = null;
    private VoltageSensor voltageSensor = null;

    private FlywheelOperate fwOperate = null;
    private RevColorSensorV3 color = null;

    boolean lastYButtonState = false;
    boolean toggleYButtonState = false;

    public void init() {

        windMill = hardwareMap.get(CRServo.class, "windmill");
        windMill.setDirection(CRServo.Direction.FORWARD);
        windMill.setPower(speeds.STOP_SPEED);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        color = hardwareMap.get(RevColorSensorV3.class, "color");


        fwOperate = new FlywheelOperate(launcher, voltageSensor, windMill, color);

        fwOperate.addTelemetry(telemetry);
        telemetry.update();

    }

    @Override
    public void loop() {

        boolean currentYButtonState = gamepad1.y;

        if (currentYButtonState && !lastYButtonState) {
            toggleYButtonState = true;
        } else {
            toggleYButtonState = false;
        }

        lastYButtonState = currentYButtonState;

        if (toggleYButtonState) {
            fwOperate.toggleSpin();
        }

        if ((gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased() )) {
            //fwOperate.fire();
        }

        fwOperate.update();

        fwOperate.addTelemetry(telemetry);
        telemetry.update();

    }

}
