using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;
using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

using CTRE.Phoenix.Motion;



namespace HERO_Serial_Example10
{
    public class Program
    {
        public static GameController _gamepad = new GameController(UsbHostDevice.GetInstance());
        public static TalonSRX axis_5 = new TalonSRX(6);

        public static double leftThrot;
        public static double rightThrot;
        public static double power;
        public static double distance_totalError = 0;
        public static void Main()
        {
            axis_5.ConfigFactoryDefault();
            //axis_5.SetSensorPhase(true);
            while (true)
            {

                double y = -1 * _gamepad.GetAxis(0) / 4;
                double x = -1 * _gamepad.GetAxis(1) / 3;

                if (x < -0.03)
                {

                }
                else if (x > 0.03)
                {

                }

                else
                    x = 0;
                //axis_5.Set(ControlMode.PercentOutput, x);
                //axis_5.Set(ControlMode.PercentOutput, pid.pid_sol_arka(0.75,0.005,0.18,x));
                //pid.parameter_changer(0.75, 0.005, 0.18);

               // Debug.Print(" POSİSYON " + axis_5.GetSelectedSensorPosition().ToString());
               // Debug.Print(" APMER " + axis_5.GetOutputCurrent().ToString());
              //ebug.Print(" HIZ " + axis_5.GetSelectedSensorVelocity().ToString());
                //double enco_rpm;

                //enco_rpm = (((double)(Program.axis_5.GetSelectedSensorVelocity()) / (850 * 12*3)));

                //Debug.Print("  ENKO RPM ==========   " + enco_rpm.ToString());

                //Debug.Print(" OUTPUTPERCENT " + axis_5.GetMotorOutputPercent().ToString());
                //Debug.Print(" VOLT " + axis_5.GetMotorOutputVoltage().ToString());
                //axis_5.Set(ControlMode.PercentOutput, pid.pid_sol_arka(pıd_coef.kP,pıd_coef.kI,pıd_coef.kD,x));
                //axis_5.Set(ControlMode.PercentOutput, pıd_coef.parameter_changer());
                pid_coef.parameter_changer(x);
               Debug.Print(" HIZ ========== " + x.ToString());
                Debug.Print(" KP " + pid_coef.kP.ToString());
                Debug.Print(" KI " + pid_coef.kI.ToString());
                Debug.Print(" KD " + pid_coef.kD.ToString());
                //Thread.Sleep(100);
                //Debug.Print(" HIZ " + axis_5.GetSelectedSensorVelocity().ToString());
                //Debug.Print(" HIZ " + axis_5.GetSelectedSensorVelocity().ToString());

                Watchdog.Feed();
            }

        }
    }
}
