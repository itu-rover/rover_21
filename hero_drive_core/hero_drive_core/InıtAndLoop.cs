using System;
using Microsoft.SPOT;
using System.Threading;
using System.Text;


using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;





namespace hero_drive_core
{
    public static class InitAndLoop
    {

        //public static Arge sol_onPID;
        //public static Arge sag_onPID;
        //public static Arge sol_arkaPID;
        //public static Arge sag_arkaPID;

        public static void Init()
        {
            SerialCom._uart.Open();
            //sol_onPID = new Arge();
            //sag_onPID = new Arge();
            //sol_arkaPID = new Arge();
            //sag_arkaPID = new Arge();

        }
        public static void Loop()
        {
            while (true)
            {
                ModeSwitch.Switcherino();
                //if(IO._gamepad.GetButton(2) == true)
                //{
                //    Direc_Drive.iyiPID_drive();
                //}
                //else
                //{
                //    IO.sag_arka.Set(ControlMode.PercentOutput, 0);
                //    IO.sag_on.Set(ControlMode.PercentOutput, 0);
                //    IO.sol_arka.Set(ControlMode.PercentOutput, 0);
                //    IO.sol_on.Set(ControlMode.PercentOutput, 0);
                Watchdog.Feed();
                //}
            }

        }
    }
}

