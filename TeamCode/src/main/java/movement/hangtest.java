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
public class hangtest extends LinearOpMode {

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

    int lexttarg;
    int rexttarg;
    int lexttargfine;
    int rexttargfine;
    double rexterr;
    double lexterr;
    double Lextpower;
    double Rextpower;
    double rextpreverr;
    double lextpreverr;

    int torquetarg;
    double torquepow;
    double torquepreverr;
    double torqueerr;

    double Kp = 0.0175;
    double Kd = 0.015;
    double torKp = 0.00005;
    double torKd = 0.00005;

    double rot;

    double offset = 0;
    double imureset = 0;

    double wristYaw;

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

        DcMotor Lext = hardwareMap.get(DcMotor.class, "Lext"); // Control hub 0
        DcMotor Rext = hardwareMap.get(DcMotor.class, "Rext"); // Control hub 1
        DcMotor Torque = hardwareMap.get(DcMotor.class, "Torque"); // Control hub 2
        DcMotor Torque2 = hardwareMap.get(DcMotor.class, "Torque2"); // Control hub 2

        CRServo george = hardwareMap.get(CRServo.class, "george");
        Servo curious = hardwareMap.get(Servo.class, "curious");
        Servo claw = hardwareMap.get(Servo.class, "Claw"); // Control hub 0
        Servo wristYawservo = hardwareMap.get(Servo.class, "WristYaw"); // Control hub 1


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
        Lext.setDirection(DcMotorSimple.Direction.REVERSE);
        Torque2.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            if (gamepad1.left_bumper && Torque2.getCurrentPosition() < -50){
                george.setPower(-1);
            } else if (gamepad1.right_bumper && Torque2.getCurrentPosition() > -12950){
                george.setPower(1);
            } else {
                george.setPower(0);
            }

            if (gamepad1.right_trigger > 0){
                curious.setPosition(1);
            } else if (gamepad1.left_trigger > 0){
                curious.setPosition(0);
            } else {
                curious.setPosition(0.5);
            }

            //-12900

            telemetry.addData("kp", Kp);
            telemetry.addData("kd", Kd);

            telemetry.addLine("Drivetrain");
            telemetry.addLine("");
            telemetry.addData("dir", dir);
            telemetry.addData("offset", offset);
            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BL Power", BL.getPower());
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Rx", gamepad1.right_stick_x);
            telemetry.addData("Ry", gamepad1.right_stick_y);

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("4 Stage Sliders");
            telemetry.addLine("");
            telemetry.addData("Left Extender Pwr", 0.01*lexterr*0.1*(lextpreverr - lexterr));
            telemetry.addData("Left Extender Enc", Lext.getCurrentPosition());
            telemetry.addData("Left Targ", lexttarg);
            telemetry.addData("Left Err", (lexttarg - Lext.getCurrentPosition()));
            telemetry.addData("Left Err Prev Diff", (lexterr - lextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addData("Right Extender Pwr", 0.01*rexterr*0.1*(rextpreverr - rexterr));
            telemetry.addData("Right Extender Enc", Rext.getCurrentPosition());
            telemetry.addData("Right Targ", rexttarg);
            telemetry.addData("Right Err", (rexttarg - Rext.getCurrentPosition()));
            telemetry.addData("Right Err Prev Diff", (lexterr - lextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Intake");
            telemetry.addLine("");
            telemetry.addData("Claw Rot", claw.getPosition());
            telemetry.addData("Wrist Yaw", wristYawservo.getPosition());
            telemetry.addData("Torque2", torquetarg);
            telemetry.addData("Torque", Torque.getPower());

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("User Inputs");
            telemetry.addLine("");
            telemetry.addData("A", buttonA);
            telemetry.addData("B", buttonB);
            telemetry.addData("X", buttonX);
            telemetry.addData("Y", buttonY);
            telemetry.addData("A2", button2A);
            telemetry.addData("B2", button2B);
            telemetry.addData("X2", button2X);
            telemetry.addData("Y2", button2Y);

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addData("Kp", torKp);
            telemetry.addData("Kd", torKd);
            telemetry.addData("torque position", Torque.getCurrentPosition());
            telemetry.addData("torque power", torquepow);

            telemetry.addData("george", george.getPower());
            telemetry.addData("curious", curious.getPosition());
            telemetry.addData("gorge", Torque2.getCurrentPosition());

            //telemetry.addData("", );

            telemetry.update();
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
