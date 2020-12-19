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
    public static class ModeSwitch
    {
        public static int mode = 0;
        public static int drive_mode = 0;
        public static int robotic_arm_drive_mode = 0;
        public static int science_drive_mode = 0;
        public static String state;
        public static Boolean bt7_prev = false;
        public static Boolean bt7_current = false;
        public static Boolean bt8_prev = false;
        public static Boolean bt8_current = false;

        public static void Switcherino()
        {

            if (null == IO._gamepad)
            {
                Debug.Print("Logitech Gamepad Initiated!");
                IO._gamepad = new GameController(UsbHostDevice.GetInstance());
            }
            bt7_current = IO._gamepad.GetButton(7);
            if (bt7_current && !bt7_prev)
            {

                if (mode == 4)
                {
                    mode = 0;
                }
                mode++;

                if (mode == 1)
                {
                    state = "drive";
                    Debug.Print(state);
                }
                else if (mode == 2)
                {
                    state = "robotic arm";
                    Debug.Print(state);
                }
                else if (mode == 3)
                {
                    state = "science";
                    Debug.Print(state);
                }
                else if (mode == 4)
                {
                    state = "STANDBY";
                    Debug.Print(state);
                }
                Thread.Sleep(10);
            }
            bt7_prev = bt7_current;

            if (mode == 1) //*************************************DRIVE FUNCTIONS*******************************************
            {
                bt8_current = IO._gamepad.GetButton(8);

                if (bt8_current && !bt8_prev)
                {

                    if (drive_mode == 4)
                    {
                        drive_mode = 0;
                    }
                    drive_mode++;

                    if (drive_mode == 1)
                    {
                        state = "Direct Drive WithOUT PID";
                        Debug.Print(state);
                    }
                    else if (drive_mode == 2)
                    {
                        state = "Direct Drive with PýD";
                        Debug.Print(state);
                    }
                    else if (drive_mode == 3)
                    {
                        state = "Serial Drive with PID ";
                        Debug.Print(state);
                    }
                    else if (drive_mode == 4)
                    {
                        state = "STANDBY";
                        Debug.Print(state);
                    }

                }
                bt8_prev = bt8_current;

                if (drive_mode == 1)
                {
                    DirectDrive._Direct__Drive();
                }
                else if (drive_mode == 2)
                {
                    //Direc_Drive._pid_Drive();
                }
                else if (drive_mode == 3)
                {
                    //if (Serial_Com._uart.BytesToRead > 0)
                    //{
                    //    Serial_Com.Serial_extract();
                    //}
                    ////PID KATSAYILARI AYRALANACAK
                    //Serial_Drive.Serial_drive();
                    ////PID KATSAYILARI AYRALANACAK
                }
                else if (drive_mode == 4)
                {
                    //Do nothing
                }

                //drive function will work in here

            }

            else if (mode == 2) //*************************************ROBOTIC ARM DRIVE FUNCTIONS*******************************************
            {

                bt8_current = IO._gamepad.GetButton(8);

                if (bt8_current && !bt8_prev)
                {

                    if (robotic_arm_drive_mode == 4)
                    {
                        robotic_arm_drive_mode = 0;
                    }
                    robotic_arm_drive_mode++;

                    if (robotic_arm_drive_mode == 1)
                    {
                        state = "Forward Kinematic Drive (Select Axis With Button = *****";
                        Debug.Print(state);
                    }
                    else if (robotic_arm_drive_mode == 2)
                    {
                        state = "Empty";
                        Debug.Print(state);
                    }
                    else if (robotic_arm_drive_mode == 3)
                    {
                        state = "Empty";
                        Debug.Print(state);
                    }
                    else if (robotic_arm_drive_mode == 4)
                    {
                        state = "STANDBY";
                        Debug.Print(state);
                    }
                    Thread.Sleep(10);


                }
                bt8_prev = bt8_current;

                if (robotic_arm_drive_mode == 1)
                {
                    RoboticArmDrive._Forward_Kinematic_Drive();
                }
                else if (robotic_arm_drive_mode == 2)
                {
                    //Empty
                }
                else if (robotic_arm_drive_mode == 3)
                {
                    //Empty
                }
                else if (robotic_arm_drive_mode == 4)
                {
                    //Do nothing
                }

                //robotic arm drive function will work in here

            }


            else if (mode == 3) //*************************************SCIENCE DRIVE FUNCTIONS*******************************************
            {

                bt8_current = IO._gamepad.GetButton(8);

                if (bt8_current && !bt8_prev)
                {

                    if (science_drive_mode == 5)
                    {
                        science_drive_mode = 0;
                    }
                    science_drive_mode++;

                    if (science_drive_mode == 1)
                    {
                        state = "Vacuum Motor Drive";
                        Debug.Print(state);
                    }
                    else if (science_drive_mode == 2)
                    {
                        state = "Ball Screw Drive";
                        Debug.Print(state);
                    }
                    else if (science_drive_mode == 3)
                    {
                        state = "kayar hazne ";
                        Debug.Print(state);
                    }
                    else if (science_drive_mode == 4)
                    {
                        state = "döner hazne";
                        Debug.Print(state);
                    }
                    else if (science_drive_mode == 5)
                    {
                        state = "STANDBY";
                        Debug.Print(state);
                    }
                    Thread.Sleep(10);

                }
                bt8_prev = bt8_current;

                if (science_drive_mode == 1)
                {
                    ScienceDrive.VacuumDrive();
                }
                else if (science_drive_mode == 2)
                {
                    ScienceDrive.BallScrewDrive();
                }
                else if (science_drive_mode == 3)
                {
                    ScienceDrive._kayar_hazne();
                }
                else if (science_drive_mode == 4)
                {
                    ScienceDrive._döner_hazne();
                }
                else if (science_drive_mode == 5)
                {
                    //Do nothing
                }

                //sciencedrive function will work in here

            }




            else if (mode == 4) //************************************STANDBY*******************************************
            {
                //do nothing
            }

        }

    }
}
