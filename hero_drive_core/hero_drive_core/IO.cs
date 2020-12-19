
using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;


using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;


namespace hero_drive_core
{
    static class IO
    {
        public static GameController _gamepad = new GameController(UsbHostDevice.GetInstance());
        //DRIVE
        public static TalonSRX sag_on = new TalonSRX(1);
        public static TalonSRX sag_arka = new TalonSRX(3);
        public static TalonSRX sol_on = new TalonSRX(2);
        public static TalonSRX sol_arka = new TalonSRX(4);


        //ARM
        public static TalonSRX axis_1 = new TalonSRX(5); //OK
        public static TalonSRX axis_2 = new TalonSRX(6); //OK
        public static TalonSRX axis_3 = new TalonSRX(7); //OK
        public static TalonSRX axis_4 = new TalonSRX(8); //OK
        public static TalonSRX axis_5 = new TalonSRX(9); //OK
        public static TalonSRX axis_6 = new TalonSRX(10); //OK
        public static TalonSRX gripper = new TalonSRX(11); //OK
        //SCIENCE
        public static TalonSRX vacuum = new TalonSRX(12);
        public static TalonSRX faulhaber = new TalonSRX(13);
    }
}
