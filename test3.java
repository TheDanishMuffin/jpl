package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "JPL_Autonomous_PID_FIXED3", group = "JPL")
public class test3 extends LinearOpMode {
    private DcMotorEx myMotor = null;
    private DcMotorEx myMotor1 = null;

    @Override
    public void runOpMode() {

        int UP_POSITION = -750;
        int DOWN_POSITION = 0;


        // 0 to 2940
        double LIFT_VEL = 450;
        double LOWER_VEL = 200;
        
        
        // motor init stuff
        myMotor = hardwareMap.get(DcMotorEx.class, "motor0");
        myMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        myMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // myMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor.setTargetPosition(0);
        myMotor1.setTargetPosition(0);

        //pidf
        PIDFCoefficients pidf = myMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        double newP = pidf.p * 1;
        double newF = 5.0;
        PIDFCoefficients newPIDF = new PIDFCoefficients(newP, pidf.i, pidf.d, newF);
        myMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);   
        
        // up?
        myMotor.setTargetPosition(UP_POSITION);
        myMotor1.setTargetPosition(UP_POSITION);    
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        // setVel should have PID for speed built in 
        myMotor.setVelocity(LIFT_VEL);
        myMotor1.setVelocity(LIFT_VEL);

        // Wait for BOTH motors to reach target
        while (opModeIsActive() && (myMotor.isBusy() || myMotor1.isBusy())) {
            telemetry.addData("M0 Pos", myMotor.getCurrentPosition());
            telemetry.addData("M1 Pos", myMotor1.getCurrentPosition());
            telemetry.update();
            }
        
        // myMotor.setPower(0);
        // myMotor1.setPower(0);
        sleep(1000);
        
        myMotor.setTargetPosition(DOWN_POSITION);
        myMotor1.setTargetPosition(DOWN_POSITION);  
        
        myMotor.setVelocity(LOWER_VEL);
        myMotor1.setVelocity(LOWER_VEL);
        
        while (opModeIsActive() && (myMotor.isBusy() || myMotor1.isBusy())) {
        telemetry.addData("M0 Pos", myMotor.getCurrentPosition());
        telemetry.addData("M1 Pos", myMotor1.getCurrentPosition());
        telemetry.update();
        }

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }
}
