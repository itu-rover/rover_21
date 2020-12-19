using System;
using Microsoft.SPOT;
using System.Threading;
using System.Text;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Motion;
using CTRE.Phoenix.MotorControl;


namespace hero_drive_core
{
    public static class DirectDrive
    {


        public static void Deadband(ref double value)
        {
            if (value < -0.00)
            {
                /* outside of deadband */
            }
            else if (value > +0.00)
            {
                /* outside of deadband */
            }
            else
            {
                /* within 10% so zero it */
                value = 0;
            }

        }
        public static double leftThrot;
        public static double rightThrot;
        public static void _Direct__Drive()
        {
            if (null == IO._gamepad)
                IO._gamepad = new GameController(UsbHostDevice.GetInstance());

            double y = -1 * IO._gamepad.GetAxis(0) / 4;
            double x = -1 * IO._gamepad.GetAxis(1) / 4;

            Deadband(ref x);
            Deadband(ref y);

            double rightThrot = (x + y);
            double leftThrot = (x - y);
            if (x > 0.333)
                x = 0.333;
            if (y > 0.333)
                y = 0.333;
            if (x < -0.333)
                x = -0.333;
            if (y < -0.333)
                y = -0.333;
            IO.sag_on.SetInverted(true);
            IO.sol_on.SetInverted(true);

            IO.sag_on.Set(ControlMode.PercentOutput, rightThrot * 0.9);
            IO.sag_arka.Set(ControlMode.PercentOutput, rightThrot * 0.9);
            IO.sol_on.Set(ControlMode.PercentOutput, leftThrot);
            IO.sol_arka.Set(ControlMode.PercentOutput, leftThrot);

            Watchdog.Feed();

            Thread.Sleep(50);


        }
        public static void _pid_Drive()
        {
            if (null == IO._gamepad)

                IO._gamepad = new GameController(UsbHostDevice.GetInstance());


            double y = IO._gamepad.GetAxis(0) / 3;
            double x = -1 * IO._gamepad.GetAxis(1) / 3;

            Deadband(ref x);
            Deadband(ref y);

            double leftThrot = (x + y);
            double rightThrot = (x - y);
            if (x > 0.333)
                x = 0.333;
            if (y > 0.333)
                y = 0.333;
            if (x < -0.333)
                x = -0.333;
            if (y < -0.333)
                y = -0.333;
            IO.sag_on.SetInverted(true);
            IO.sol_on.SetInverted(true);

            //IO.sag_on.Set(ControlMode.PercentOutput, Pid_Calculate.pid_sag_on(0.15, 0.00, 0.07, rightThrot));
            //IO.sag_arka.Set(ControlMode.PercentOutput, Pid_Calculate.pid_sag_arka(0.15, 0.01, 0.07, rightThrot));
            //IO.sol_arka.Set(ControlMode.PercentOutput, Pid_Calculate.pid_sol_arka(0.15, 0.01, 0.07, leftThrot));
            //IO.sol_on.Set(ControlMode.PercentOutput, Pid_Calculate.pid_sol_on(0.15, 0.01, 0.007, leftThrot));

        }
        //public static void iyiPID_drive()
        //{
        //    double y = -1 * IO._gamepad.GetAxis(0) / 4;
        //    double x = -1 * IO._gamepad.GetAxis(1) / 4;
        //    double rightThrot = (x + y) * 0.93;
        //    double leftThrot = (x - y);



        //    IO.sol_on.Set(ControlMode.PercentOutput, Init_Loops.sol_onPID.iyiPID(leftThrot, Encoder.do_encoder(IO.sol_on)));
        //    IO.sol_arka.Set(ControlMode.PercentOutput, Init_Loops.sol_arkaPID.iyiPID(leftThrot, Encoder.do_encoder(IO.sol_arka)));
        //    IO.sag_arka.Set(ControlMode.PercentOutput, Init_Loops.sag_arkaPID.iyiPID(leftThrot, Encoder.do_encoder(IO.sag_arka)));
        //    IO.sag_on.Set(ControlMode.PercentOutput, Init_Loops.sag_onPID.iyiPID(leftThrot, Encoder.do_encoder(IO.sag_on)));
        //    Thread.Sleep(20);
        //    Watchdog.Feed();

        //}
    }
}
