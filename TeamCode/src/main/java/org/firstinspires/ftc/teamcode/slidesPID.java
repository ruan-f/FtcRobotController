package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

 class SlidesPID extends LinearOpMode {

    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double ticksPerRotation = 384.5;
    double circumference = 2*Math.PI;
    double desiredInches = 10;

    double ticks = (desiredInches/circumference)*ticksPerRotation;
    public void runOpMode() throws InterruptedException {
        DcMotor slidesMotor = hardwareMap.dcMotor.get("slidesMotor");

        waitForStart();

        while(opModeIsActive()){
            slidesMotor.setPower(PIDcontrol(ticks, slidesMotor.getCurrentPosition()));
        }
    }

    public double PIDcontrol (double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError)/ timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error* kp) + (derivative*kd) + (integralSum*ki);
        return output;
    }
}
