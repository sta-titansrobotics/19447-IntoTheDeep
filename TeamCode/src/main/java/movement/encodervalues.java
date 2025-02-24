package movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class encodervalues extends LinearOpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 0;
    int buttonY = 0;

    boolean butAcheck = false;
    boolean butYcheck = false;
    boolean butXcheck = false;
    boolean butBcheck = false;
    boolean but2Acheck = false;
    boolean but2Ycheck = false;
    boolean but2Xcheck = false;
    boolean but2Bcheck = false;

    double prevtime;

    static double dir;
    static double mag;
    static double pi = Math.PI;

    int XAxistarg;
    int XAxistargfine;
    double XAxiserr;
    double XAxispower;
    double XAxispreverr;

    double XKp = 0.011;
    double XKd = 0.001;
    double HlKp = 0.0125;
    double HlKd = 0.0015;

    int VLexttarg;
    int VRexttarg;
    int VLexttargfine;
    int VRexttargfine;
    double VRexterr;
    double VLexterr;
    double VLextpower;
    double VRextpower;
    double VRextpreverr;
    double VLextpreverr;

    double VKp = 0.01;
    double VKd = 0.0015;

    double rot;

    double offset = 0;
    double imureset = 0;

    double toepos = 0.97;

    double xpos = 0;
    double ypos = 0;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angle;

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Moving
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // Expansion hub 
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL"); // Expansion hub 
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR"); // Expantion hub 
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR"); // Expantion hub

        DcMotor HRext = hardwareMap.get(DcMotor.class, "hrext"); // Expantion hub
        DcMotor HLext = hardwareMap.get(DcMotor.class, "hlext"); // Expantion hub

        DcMotor VRext = hardwareMap.get(DcMotor.class, "vrext"); // Expantion hub
        DcMotor VLext = hardwareMap.get(DcMotor.class, "vlext"); // Expantion hub

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(parameters);

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        HRext.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo claw = hardwareMap.get(Servo.class, "Claw");



        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            ypos = FR.getCurrentPosition();
            xpos = BR.getCurrentPosition();

            XAxistarg = (int) clamp(XAxistarg, 10, 1950);

            XAxiserr = XAxistarg - xpos;

            // actual pd calculations
            XAxispower = XAxiserr*XKp+(XAxiserr - XAxispreverr)*XKd;

            // getting the previous error
            XAxispreverr = (XAxistarg - xpos);

            // actually setting the motor power
            //HRext.setPower(clamp(XAxispower, -1, 1));

            if (gamepad2.y && !butYcheck) {
                buttonY += 1;
                butYcheck = true;
            }

            if (!gamepad2.y) {
                butYcheck = false;
            }

            if (butYcheck) {
                if (buttonY % 2 == 1) {
                    XAxistarg = 1900;
                } else {
                    XAxistarg = 10;
                }
            }

            telemetry.addData("Hor Right Extender Pwr", HRext.getPower());
            telemetry.addData("Hor Right Extender Enc", HRext.getCurrentPosition());

            telemetry.addData("Hor Left Extender Pwr", HLext.getPower());
            telemetry.addData("Hor Left Extender Enc", HLext.getCurrentPosition());

            telemetry.addData("vert Right Extender Pwr", VRext.getPower());
            telemetry.addData("vert Right Extender Enc", VRext.getCurrentPosition());

            telemetry.addData("vert Left Extender Pwr", VLext.getPower());
            telemetry.addData("vert Left Extender Enc", VLext.getCurrentPosition());

            telemetry.addData("Y Enc", ypos);
            telemetry.addData("X Enc", xpos);

            telemetry.addData("Xtarg", XAxistarg);
            telemetry.addData("Xerr", XAxiserr);
            telemetry.addData("Xpower", XAxispower);
            telemetry.addData("Xpreverr", XAxispreverr);
            telemetry.addData("XKp", XKp);
            telemetry.addData("XKd", XKd);

            //telemetry.addData("", );

            telemetry.update();

            telemetry.addData("claw", claw.getPosition());

            claw.setPosition(gamepad1.right_trigger);
        }
    }
    private double getAngle()
    {
        //this converts the imu's outputs from -180 to 180 into an output of 0 to 360

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle += deltaAngle;

        lastAngles = angles;

        return angle;
    }
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
