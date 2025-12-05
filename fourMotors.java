package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MOTOR_ALIGN", group = "JPL")
public class shaftAlign extends LinearOpMode {

    // --- CONFIGURATION ---
    // Change this value to 0, 1, 2, or 3 to test that specific motor
    final int TEST_MOTOR_INDEX = 2; 
    // ---------------------

    private DcMotorEx myMotor0 = null;
    private DcMotorEx myMotor1 = null;
    private DcMotorEx myMotor2 = null;
    private DcMotorEx myMotor3 = null;

    @Override
    public void runOpMode() {

        int UP_POSITION = -1000; 
        int DOWN_POSITION = 0;
        double ratio_offset = 1.5;

        double LIFT_VEL = 200; 
        double LOWER_VEL = 150;
        
        // Initialize ALL motors (prevents configuration errors)
        myMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        myMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        myMotor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        
        myMotor0.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        myMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Reset all encoders
        myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set zero power behavior (Keep them stiff so we can see if the active one drags)
        myMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PIDF Setup
        PIDFCoefficients pidf40 = myMotor0.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf60 = myMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients newPIDF40 = new PIDFCoefficients(pidf40.p, pidf40.i, pidf40.d, pidf40.f + 15);
        PIDFCoefficients newPIDF60 = new PIDFCoefficients(pidf60.p, pidf60.i, pidf60.d, pidf60.f + 15);
        
        myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF40);
        myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF60);
        myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF60);
        myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF40);

        // Select the active motor for the user
        DcMotorEx activeMotor = null;
        double targetVel = 0;
        int targetPos = 0;
        
        switch (TEST_MOTOR_INDEX) {
            case 0: activeMotor = myMotor0; break;
            case 1: activeMotor = myMotor1; break;
            case 2: activeMotor = myMotor2; break;
            case 3: activeMotor = myMotor3; break;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Testing Motor", TEST_MOTOR_INDEX);
        telemetry.update();
        waitForStart();

        // --- MOVE UP ---
        if (activeMotor != null) {
            // Calculate specific target based on gear ratio (40:1 vs 60:1)
            // Motors 1 and 2 are 60:1 and need the offset
            boolean is60to1 = (TEST_MOTOR_INDEX == 1 || TEST_MOTOR_INDEX == 2);
            
            targetPos = is60to1 ? (int)(UP_POSITION * ratio_offset) : UP_POSITION;
            targetVel = is60to1 ? (int)(LIFT_VEL * ratio_offset) : LIFT_VEL;

            activeMotor.setTargetPosition(targetPos);
            activeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            activeMotor.setVelocity(targetVel);

            // Wait for ONLY the active motor
            while (opModeIsActive() && activeMotor.isBusy()) {
                telemetry.addData("Testing Motor", TEST_MOTOR_INDEX);
                telemetry.addData("Current Pos", activeMotor.getCurrentPosition());
                telemetry.addData("Target Pos", targetPos);
                telemetry.update();
            }
        }
        
        sleep(800);

        // --- MOVE DOWN ---
        if (activeMotor != null) {
            // Apply Down PIDF
            PIDFCoefficients downPIDF = (TEST_MOTOR_INDEX == 1 || TEST_MOTOR_INDEX == 2) ? 
                new PIDFCoefficients(pidf60.p, pidf60.i, pidf60.d, pidf60.f + 15) : 
                new PIDFCoefficients(pidf40.p, pidf40.i, pidf40.d, pidf40.f + 15);
                
            activeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF);

            // Calculate velocities/pos for down
            boolean is60to1 = (TEST_MOTOR_INDEX == 1 || TEST_MOTOR_INDEX == 2);
            targetVel = is60to1 ? (int)(LOWER_VEL * ratio_offset) : LOWER_VEL;
            
            activeMotor.setTargetPosition(DOWN_POSITION);
            activeMotor.setVelocity(targetVel);
            
            // Wait for ONLY the active motor
            while (opModeIsActive() && activeMotor.isBusy()) {
                telemetry.addData("Testing Motor", TEST_MOTOR_INDEX);
                telemetry.addData("Current Pos", activeMotor.getCurrentPosition());
                telemetry.update();
            }
        }

        telemetry.addData("Status", "Single Motor Test Done!");
        telemetry.update();
    }
}
