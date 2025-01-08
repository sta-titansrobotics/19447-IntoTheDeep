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

        HRext.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            telemetry.addData("Hor Right Extender Pwr", HRext.getPower());
            telemetry.addData("Hor Right Extender Enc", HRext.getCurrentPosition());

            telemetry.addData("Hor Left Extender Pwr", HLext.getPower());
            telemetry.addData("Hor Left Extender Enc", HLext.getCurrentPosition());

            telemetry.addData("vert Right Extender Pwr", VRext.getPower());
            telemetry.addData("vert Right Extender Enc", VRext.getCurrentPosition());

            telemetry.addData("vert Left Extender Pwr", VLext.getPower());
            telemetry.addData("vert Left Extender Enc", VLext.getCurrentPosition());

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
