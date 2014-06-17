using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Net;
using System.Net.Sockets;
using Microsoft.Kinect;
using System.IO;

namespace KinectSkeletonViewer
{
    class Mirroring
    {
        //Finds Midpoints between two points with x,y,z coordinates
        public int[] Midpoint(int[] point_1, int[] point_2)
        {
            int[] mdpt = new int[3];
            for (int i = 0; i < 3; i++)
            {
                mdpt[i] = (point_1[i] + point_2[i]) / 2;
            }
            return mdpt;
        }

        // Finds the vector pointing from point_1 to point_2 with x y z coordinates 
        public int[] Vector(int[] point_1, int[] point_2)
        {

            
           //int[] distance_vector = { Math.Abs(point_1[0] - point_2[0]), Math.Abs(point_1[1] - point_2[1]), Math.Abs(point_1[2] - point_2[2]) };
            int[] distance_vector = { point_2[0] - point_1[0], point_2[1] - point_1[1],point_2[2] - point_1[2] };
           /* if (point_2[1] < point_1[1])
            {
                distance_vector[1] = -distance_vector[1];

            }*/
            return distance_vector;
        }

        // Distance formula to find the distance between point 1 and point 2
        public double DistanceFormula(int[] point_1, int[] point_2)
        {
            double distance = 0;
            distance = Math.Sqrt(Math.Pow((point_2[0] - point_1[0]), 2) + Math.Pow((point_2[1] - point_1[1]), 2) + Math.Pow((point_2[2] - point_1[2]), 2));
            return distance;
        }

        // Computes the Dot Product
        public int DotProduct(int[] line_1, int[] line_2)
        {
            int scalar = line_1[0] * line_2[0] + line_1[1] * line_2[1] + line_1[2] * line_2[2];
            return scalar;
        }

        // Finds magnitude of a vector
        public double Magnitude(int[] line_1)
        {
            double mag_res;
            mag_res = Math.Sqrt(Math.Pow(line_1[0], 2) + Math.Pow(line_1[1], 2) + Math.Pow(line_1[2], 2));
            return mag_res;
        }

        // Convert the position to duty cycle
        public double dutyCycle(double angle)
        {
            double dc;
            

            dc = 11.5 / 180 * (angle) + 1;

            return dc;
        }

        // Find Direction of Vector
        public double[] DirectionVec(int[] Input_Vector)
        {
            double vec_mag = Magnitude(Input_Vector);
            double[] dbl_Input_Vector = {0,0,0};
            for (int i = 0; i < Input_Vector.Length; i++)
            {
                dbl_Input_Vector[i] = Input_Vector[i] / vec_mag;
            }
            return dbl_Input_Vector;
        }

        //Finds the Scaled Value of Degrees for the Pololu Servo Controller
        public byte[] PololuPosition(int degrees , int i)
        {
            //Finds the scale for the degrees 180/127 (which 127 is the largest value for the Pololu controller) = 1.42
            int scaled_degrees = Convert.ToInt32(1.42*degrees);
            byte[] b_scaled_degrees = {0,0};

            if (scaled_degrees >= 128)
            {
                scaled_degrees = scaled_degrees - 128;
                b_scaled_degrees[0] = 1;
            }
            if (scaled_degrees > 127)
            {
                scaled_degrees = 127;
            }


            try
            {
                b_scaled_degrees[1] = Convert.ToByte(scaled_degrees);
            }
            catch (System.OverflowException e){
              //  MessageBox.Show(degrees.ToString(), i.ToString());
             
                

            }
                return b_scaled_degrees;
            
        }

    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor myKinect;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

            // Check to see if a Kinect is available
            if (  KinectSensor.KinectSensors.Count == 0)
            {
               // MessageBox.Show("No Kinects detected", "Camera Viewer"); COMMENTED OUT FOR Pi
                Application.Current.Shutdown();
          
           }

            // Get the first Kinect on the computer
            myKinect = KinectSensor.KinectSensors[0];

            // Start the Kinect running and select the depth camera
            for (int i = 0; i < 30; i++)
            {

                try
                {
                    myKinect.SkeletonStream.Enable();
                    myKinect.Start();
                }

                catch
                {
                    //TRY THIS TO RESTART UNTIL KINECT WORKS
                    //System.Diagnostics.Process.Start(Application.ExecutablePath); // to start new instance of application
                   // this.Close(); //to turn off current app

                  //  MessageBox.Show("Kinect initialise failed", "Camera Viewer");
                  //  Application.Current.Shutdown();
                  //  Application.Current.Startup();

                }
                
            }
            //  MessageBox.Show("here"); COMMENTED OUT FOR Pi

            // connect a handler to the event that fires when new frames are available

            myKinect.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(myKinect_SkeletonFrameReady);

        }

        // Brush skeletonBrush = new SolidColorBrush(Colors.Red); COMMENTED OUT FOR Pi

        //draws the lines on the viewer
        //void addLine(Joint j1, Joint j2)   COMMENTED OUT FOR Pi
        //{
        //    Line boneLine = new Line();
        //    boneLine.Stroke = skeletonBrush;
        //    boneLine.StrokeThickness = 5;
        //    float j1X, j1Y;

        //    DepthImagePoint j1P = myKinect.MapSkeletonPointToDepth(j1.Position, DepthImageFormat.Resolution640x480Fps30);
        //    boneLine.X1 = j1P.X;
        //    boneLine.Y1 = j1P.Y;

        //    DepthImagePoint j2P = myKinect.MapSkeletonPointToDepth(j2.Position, DepthImageFormat.Resolution640x480Fps30);
        //    boneLine.X2 = j2P.X;
        //    boneLine.Y2 = j2P.Y;

        //    skeletonCanvas.Children.Add(boneLine);
        //}

        void myKinect_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {

            string message = "No Skeleton Data";
            string qualityMessage = "";

            // Remove the old skeleton
            skeletonCanvas.Children.Clear();
            // Brush brush = new SolidColorBrush(Colors.Red); COMMENTED OUT FOR Pi

            SkeletonFrame frame = e.OpenSkeletonFrame();

            if (frame == null) return;

            Skeleton[] skeletons = new Skeleton[frame.SkeletonArrayLength];
            frame.CopySkeletonDataTo(skeletons);

            foreach (Skeleton skeleton in skeletons)
            {
                if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                {
                    TcpClient tcpclnt = new TcpClient();  

                    tcpclnt.Connect("192.168.1.125", 8001);

                    // Head
                    Joint headJoint = skeleton.Joints[JointType.Head];

                    // Left side of body
                    Joint shouldercenter = skeleton.Joints[JointType.ShoulderCenter];
                    SkeletonPoint shouldercentposition = shouldercenter.Position;
                    Joint shoulderleft = skeleton.Joints[JointType.ShoulderLeft];
                    SkeletonPoint shoulderleftposition = shoulderleft.Position;
                    Joint wristLeft = skeleton.Joints[JointType.WristLeft];
                    SkeletonPoint wristleftposition = wristLeft.Position;
                    Joint elbowLeft = skeleton.Joints[JointType.ElbowLeft];
                    SkeletonPoint elbowleftposition = elbowLeft.Position;
                    Joint handLeft = skeleton.Joints[JointType.HandLeft];
                    SkeletonPoint handleftposition = handLeft.Position;

                    // Right side of body
                    Joint shoulderright = skeleton.Joints[JointType.ShoulderRight];
                    SkeletonPoint shoulderrightposition = shoulderright.Position;
                    Joint wristRight = skeleton.Joints[JointType.WristRight];
                    SkeletonPoint wristrightposition = wristRight.Position;
                    Joint elbowRight = skeleton.Joints[JointType.ElbowRight];
                    SkeletonPoint elbowrightposition = elbowRight.Position;
                    Joint handRight = skeleton.Joints[JointType.HandRight];
                    SkeletonPoint handrightposition = handRight.Position;
                    SkeletonPoint headPosition = headJoint.Position;

                    // str = headPosition.X.ToString();
                    //left side of body
                    int[] shouldercentarr = new int[3];
                    shouldercentarr[0] = Convert.ToInt32(shouldercentposition.X * 1000);
                    shouldercentarr[1] = Convert.ToInt32(shouldercentposition.Y * 1000);
                    shouldercentarr[2] = Convert.ToInt32(shouldercentposition.Z * 1000);

                    int[] shoulderleftarr = new int[3];
                    shoulderleftarr[0] = Convert.ToInt32(shoulderleftposition.X * 1000);
                    shoulderleftarr[1] = Convert.ToInt32(shoulderleftposition.Y * 1000);
                    shoulderleftarr[2] = Convert.ToInt32(shoulderleftposition.Z * 1000);

                    int[] wristleftarr = new int[3];
                    wristleftarr[0] = Convert.ToInt32(wristleftposition.X * 1000);
                    wristleftarr[1] = Convert.ToInt32(wristleftposition.Y * 1000);
                    wristleftarr[2] = Convert.ToInt32(wristleftposition.Z * 1000);

                    int[] elbowleftarr = new int[3];
                    elbowleftarr[0] = Convert.ToInt32(elbowleftposition.X * 1000);
                    elbowleftarr[1] = Convert.ToInt32(elbowleftposition.Y * 1000);
                    elbowleftarr[2] = Convert.ToInt32(elbowleftposition.Z * 1000);

                    int[] handleftarr = new int[3];
                    handleftarr[0] = Convert.ToInt32(handleftposition.X * 1000);
                    handleftarr[1] = Convert.ToInt32(handleftposition.Y * 1000);
                    handleftarr[2] = Convert.ToInt32(handleftposition.Z * 1000);

                    //right side
                    int[] shoulderrightarr = new int[3];
                    shoulderrightarr[0] = Convert.ToInt32(shoulderrightposition.X * 1000);
                    shoulderrightarr[1] = Convert.ToInt32(shoulderrightposition.Y * 1000);
                    shoulderrightarr[2] = Convert.ToInt32(shoulderrightposition.Z * 1000);

                    int[] wristrightarr = new int[3];
                    wristrightarr[0] = Convert.ToInt32(wristrightposition.X * 1000);
                    wristrightarr[1] = Convert.ToInt32(wristrightposition.Y * 1000);
                    wristrightarr[2] = Convert.ToInt32(wristrightposition.Z * 1000);

                    int[] elbowrightarr = new int[3];
                    elbowrightarr[0] = Convert.ToInt32(elbowrightposition.X * 1000);
                    elbowrightarr[1] = Convert.ToInt32(elbowrightposition.Y * 1000);
                    elbowrightarr[2] = Convert.ToInt32(elbowrightposition.Z * 1000);

                    int[] handrightarr = new int[3];
                    handrightarr[0] = Convert.ToInt32(handrightposition.X * 1000);
                    handrightarr[1] = Convert.ToInt32(handrightposition.Y * 1000);
                    handrightarr[2] = Convert.ToInt32(handrightposition.Z * 1000);

                    Mirroring n = new Mirroring();
                    int[] right_shoulder = shoulderrightarr;              // Right Arm
                    int[] right_elbow = elbowrightarr;
                    int[] right_wrist = wristrightarr;
                    int[] right_hand = handrightarr;

                    int[] left_shoulder = shoulderleftarr;                // Left Arm
                    int[] left_elbow = elbowleftarr;
                    int[] left_wrist = wristleftarr;
                    int[] left_hand = handleftarr;


                    int[] r_mdpt_shoulder_wrist = new int[3];             // Midpoint of right shoulder and wrist
                    int[] l_mdpt_shoulder_wrist = new int[3];             // Midpoint of left shoulder and wrist

                    int[] elbow_wrist_vector = new int[3];                // Vector from elbow to wrist
                    int[] r_shoulder_elbow_vector = new int[3];           // Vector from shoulder to elbow


                    double right_shoulder_rot_angle = 0;                  // Shoulder rotation angle
                    double right_shoulder_up_down_angle = 180;            // Shoulder Up Down Angle
                    double right_elbow_angle = 0;                         // Angle between shoulder and wrist, vertex at elbow
                    double right_elbow_rot_angle = 0;                     // Elbow Rotation Angle
                    double right_elbow_open_close_angle = 0;              // Elbow Open Close Angle

                    double r_shoulder_rot_dc = 0;
                    double r_shoulder_up_down_dc = 0;
                    double r_elbow_rot_dc = 0;
                    double r_elbow_open_close_dc = 0;
                    double r_wrist_up_down_dc = 0;

                    byte[] r_shoulder_up_down_pos = { 0, 0 };                     // Number to be sent to the Pi for the Pololu Controller
                    byte[] r_shoulder_rot_pos = { 0, 0 };                         // Number to be sent to the Pi for the Pololu Controller
                    byte[] r_elbow_rot_pos = { 0, 0 };                            // Number to be sent to the Pi for the Pololu Controller
                    byte[] r_elbow_open_close_pos = { 0, 0 };                     // Number to be sent to the Pi for the Pololu Controller
                    byte[] r_wrist_up_down_pos = { 0, 0 };                        // Number to be sent to the Pi for the Pololu Controller

                    double left_shoulder_rot_angle = 0;                  // Shoulder rotation angle
                    double left_shoulder_up_down_angle = 180;            // Shoulder Up Down Angle
                    double left_elbow_angle = 0;                         // Angle between shoulder and wrist, vertex at elbow
                    double left_elbow_rot_angle = 0;                     // Elbow Rotation Angle
                    double left_elbow_open_close_angle = 0;              // Elbow Open Close Angle

                    byte[] l_shoulder_up_down_pos = { 0, 0 };                     // Number to be sent to the Pi for the Pololu Controller
                    byte[] l_shoulder_rot_pos = { 0, 0 };                         // Number to be sent to the Pi for the Pololu Controller
                    byte[] l_elbow_rot_pos = { 0, 0 };                            // Number to be sent to the Pi for the Pololu Controller
                    byte[] l_elbow_open_close_pos = { 0, 0 };                     // Number to be sent to the Pi for the Pololu Controller
                    byte[] l_wrist_up_down_pos = {0,0};                        // Number to be sent to the Pi for the Pololu Controller

                    int[] l_shoulder_elbow_vector = { 0, 0, 0 };
                    double l_shoulder_rot_dc = 0;
                    double l_shoulder_up_down_dc = 0;
                    double l_elbow_rot_dc = 0;
                    double l_elbow_open_close_dc = 0;

                    bool r_elbow_flag = false;
                    bool l_elbow_flag = false;

                    r_shoulder_elbow_vector = n.Vector(right_shoulder, right_elbow);


                    // !!!!!!RIGHT ARM !!!!!!

                    //COMMENTED OUT FOR SPEEED TESTING 

                    // Finds the angles for the right shoulder
                    if (r_shoulder_elbow_vector[0] != 0 && r_shoulder_elbow_vector[1] != 0)
                    {

                        if (r_shoulder_elbow_vector[0] < 0 && r_shoulder_elbow_vector[1] > 0)
                        {
                            right_shoulder_up_down_angle = 0;
                        }
                        else if (r_shoulder_elbow_vector[0] < 0 && r_shoulder_elbow_vector[1] < 0)
                        {
                            right_shoulder_up_down_angle = 180;
                        }
                        else
                        {
                            right_shoulder_up_down_angle = Math.Atan2(r_shoulder_elbow_vector[0], r_shoulder_elbow_vector[1]) * 180 / (Math.PI);
                        }
                        r_shoulder_up_down_dc = n.dutyCycle(right_shoulder_up_down_angle);

                        if (right_shoulder_up_down_angle < 0)
                        {
                            right_shoulder_up_down_angle = 0;
                        }

                        if (right_shoulder_up_down_angle > 180)
                        {
                            right_shoulder_up_down_angle = 180;
                        }

                        // Finds Right Shoulder Rotation Angle
                        if (right_elbow[0] < right_shoulder[0] && right_elbow[2] < right_wrist[2])
                        {
                            right_shoulder_rot_angle = 0;
                        }
                        else if (right_elbow[0] > right_shoulder[0] && right_elbow[2] < right_wrist[2])
                        {
                            right_shoulder_rot_angle = 180;
                        }

                        else
                        {
                            right_shoulder_rot_angle = Math.Atan2(r_shoulder_elbow_vector[2], r_shoulder_elbow_vector[0]) * 180 / (Math.PI) + 90;
                        }
                       // MessageBox.Show(right_shoulder_rot_angle.ToString());
                        r_shoulder_rot_dc = n.dutyCycle(right_shoulder_rot_angle);

                        if (right_shoulder_rot_angle < 0)
                        {
                            right_shoulder_rot_angle = 0;
                        }

                        if (right_shoulder_rot_angle > 180)
                        {
                            right_shoulder_rot_angle = 180;
                        }
                    }
                    else
                    { }


                    // This if statement is making sure not dividing by 0
                    if (r_shoulder_elbow_vector[0] == 0 && r_shoulder_elbow_vector[1] == 0 && r_shoulder_elbow_vector[2] == 0)
                    {
                        r_elbow_flag = true;
                    }

                    // The arrays below are used to find the rotational angle for the right angle using a reference point, which changes when the right elbow changes 
                    // Finds a reference point to used to find the elbow rotation angle - used with the wrist location
                    int[] right_wrist_reference_point = { right_elbow[0], right_elbow[1] - Convert.ToInt32(n.DistanceFormula(right_elbow, right_wrist)), right_elbow[2] };

                    // Distance from the elbow to the reference point
                    int[] distance_elbow_reference = n.Vector(right_wrist_reference_point, right_wrist);

                    // The variable representing the vector from the elbow to the wrist
                    int[] right_elbow_wrist_vector = n.Vector(right_elbow, right_wrist);

                    // Computations to find the elbow rotation angle
                    right_elbow_rot_angle = Math.Atan2(distance_elbow_reference[2], right_elbow_wrist_vector[1]) * 180 / (2 * Math.PI);
                    right_elbow_rot_angle = right_elbow_rot_angle * 2;

                    // Only allows angle to be in increments of 5 degrees - decreases bouncing for the angle
                    right_elbow_rot_angle = 5 * Convert.ToInt32(right_elbow_rot_angle / 5) + 180;

                    right_elbow_angle = Math.Atan2((n.DistanceFormula(r_mdpt_shoulder_wrist, right_shoulder)), (n.DistanceFormula(r_mdpt_shoulder_wrist, right_elbow))) * 180 / (2 * Math.PI);

                    // Sets the bounds for movements for MIMER - for right_elbow_rot_angle
                    if (right_wrist[2] > right_elbow[2] && right_wrist[1] < right_elbow[1])
                    {
                        right_elbow_rot_angle = 0;
                    }
                    else if (right_wrist[2] > right_elbow[2] && right_wrist[1] > right_elbow[1])
                    {
                        right_elbow_rot_angle = 180;
                    }

                    r_elbow_rot_dc = n.dutyCycle(right_elbow_rot_angle);

                    if (right_elbow_rot_angle < 0)
                    {
                        right_elbow_rot_angle = 0;
                    }

                    if (right_elbow_rot_angle > 180)
                    {
                        right_elbow_rot_angle = 180;
                    }
                    // Finds the Elbow Open and Close Angle - uses the midpoint of the shoulder and wrist to find a right angle to take the tangent.
                    r_mdpt_shoulder_wrist = n.Midpoint(right_shoulder, right_wrist);
                    double r_dist_shoulder_mdpt_sw = n.DistanceFormula(right_shoulder, r_mdpt_shoulder_wrist);
                    double r_dist_elbow_mdpt_sw = n.DistanceFormula(right_elbow, r_mdpt_shoulder_wrist);
                    right_elbow_open_close_angle = 2 * Math.Atan2(r_dist_shoulder_mdpt_sw, r_dist_elbow_mdpt_sw) * 180 / Math.PI;                            // r_mdpt_shoulder_wrist = n.Midpoint(right_shoulder, right_wrist); unneeded at the moment
                    r_elbow_open_close_dc = n.dutyCycle(right_elbow_open_close_angle);
                    if (right_elbow_open_close_angle < 0)
                    {
                        right_elbow_open_close_angle = 0;
                    }

                    if (right_elbow_open_close_angle > 180)
                    {
                        right_elbow_open_close_angle = 180;
                    }
                    // Finds the Wrist Up Down Angle
                    //double[] r_dir_elbow_wrist = {0,0,0};               // Direction of elbow wrist vector to find reference point
                    //int[] r_wrist_ref_pt = { 0, 0, 0 };                 // Wrist reference point to make an isosceles triangle with wrist and hand
                    //int[] r_wrist_mdpt_wrw = { 0, 0, 0 };               // Midpoint between right wrist and the right wrist refernce point
                    //double r_dist_hand_mdpt_rwh = 0;                    // Distance between hand and midpoint of hand and hand reference point


                    //r_dir_elbow_wrist = n.DirectionVec(right_elbow_wrist_vector);

                    //for (int i = 0; i < 3; i++)                         // Finds reference point for the angle of wrist
                    //{
                    //    r_wrist_ref_pt[i] = Convert.ToInt32(n.DistanceFormula(right_wrist, right_hand) * r_dir_elbow_wrist[i]);
                    //}

                    //r_wrist_mdpt_wrw = n.Midpoint(right_hand, r_wrist_ref_pt);
                    //r_dist_hand_mdpt_rwh = n.DistanceFormula(right_hand, r_wrist_mdpt_wrw);

                    //r_wrist_up_down_angle = 2 * Math.Atan2(Convert.ToDouble(r_dist_hand_mdpt_rwh), n.DistanceFormula(right_wrist, right_hand)) * 180 / Math.PI;
                    //r_wrist_up_down_dc = n.dutyCycle(r_wrist_up_down_angle);


                    // Fix - adjust so motor moves to 180 deg when outside bounds
                    //if (right_current_elbow_angle + right_elbow_angle >= 180)
                    //{
                    //    right_current_elbow_angle = 180;
                    //    right_current_elbow_angle = n.dutyCycle(right_current_elbow_angle);// convert to duty cycle 
                    //}
                    //else if (right_current_elbow_angle + right_elbow_angle <= 0)
                    //{
                    //    right_current_elbow_angle = 0;
                    //    right_current_elbow_angle = n.dutyCycle(right_current_elbow_angle);// convert to duty cycle 
                    //}
                    //else
                    //{
                    //    right_current_elbow_angle = right_current_elbow_angle + right_elbow_angle;
                    //    right_current_elbow_angle = n.dutyCycle(right_current_elbow_angle);// convert to duty cycle 
                    //}

                    
                    // !!!!!!LEFT ARM !!!!!!


                    // Finds the angles for the left shoulder
                    if (l_shoulder_elbow_vector[0] != 0 && l_shoulder_elbow_vector[1] != 0)
                    {
                        // Finds the angle for the left shoulder up down angle
                        if (l_shoulder_elbow_vector[0] > 0 && l_shoulder_elbow_vector[1] > 0)
                        {
                            left_shoulder_up_down_angle = 0;
                        }
                        else if (l_shoulder_elbow_vector[0] > 0 && l_shoulder_elbow_vector[1] < 0)
                        {
                            left_shoulder_up_down_angle = 180;
                        }
                        else
                        {
                            left_shoulder_up_down_angle = Math.Atan2(l_shoulder_elbow_vector[0], l_shoulder_elbow_vector[1]) * 180 / (Math.PI);
                        }
                        l_shoulder_up_down_dc = n.dutyCycle(Math.Abs(left_shoulder_up_down_angle));

                        if (left_shoulder_up_down_angle < 0)
                        {
                            left_shoulder_up_down_angle = 0;
                        }

                        if (left_shoulder_up_down_angle > 180)
                        {
                            left_shoulder_up_down_angle = 180;
                        }

                        // Finds Left Shoulder Rotation Angle

                        if (left_elbow[0] < left_shoulder[0] && left_elbow[2] < left_wrist[2])
                        {
                            left_shoulder_rot_angle = 0;
                        }
                        else if (left_elbow[0] > left_shoulder[0] && left_elbow[2] < left_wrist[2])
                        {
                            left_shoulder_rot_angle = 180;
                        }

                        else
                        {
                            left_shoulder_rot_angle = Math.Atan2(l_shoulder_elbow_vector[2], l_shoulder_elbow_vector[0]) * 180 / (Math.PI) + 90;
                        }
                        l_shoulder_rot_dc = n.dutyCycle(left_shoulder_rot_angle);

                        if (left_shoulder_rot_angle < 0)
                        {
                            left_shoulder_rot_angle = 0;
                        }

                        if (left_shoulder_rot_angle > 180)
                        {
                            left_shoulder_rot_angle = 180;
                        }

                    }
                    else { }


                    // This if statement is making sure not dividing by 0
                    if (l_shoulder_elbow_vector[0] == 0 && l_shoulder_elbow_vector[1] == 0 && l_shoulder_elbow_vector[2] == 0)
                    {
                        l_elbow_flag = true;
                    }

                    // The arrays below are used to find the rotational angle for the right angle using a reference point, which changes when the right elbow changes 
                    // Finds a reference point to used to find the elbow rotation angle - used with the wrist location
                    int[] left_wrist_reference_point = { left_elbow[0], left_elbow[1] - Convert.ToInt32(n.DistanceFormula(left_elbow, left_wrist)), left_elbow[2] };

                    // Distance from the elbow to the reference point
                    distance_elbow_reference = n.Vector(left_wrist_reference_point, left_wrist);

                    // The variable representing the vector from the elbow to the wrist
                    int[] left_elbow_wrist_vector = n.Vector(left_elbow, left_wrist);

                    // Computations to find the elbow rotation angle
                    left_elbow_rot_angle = Math.Atan2(distance_elbow_reference[2], left_elbow_wrist_vector[1]) * 180 / (2 * Math.PI);
                    left_elbow_rot_angle = left_elbow_rot_angle * 2;

                    // Only allows angle to be in increments of 5 degrees - decreases bouncing for the angle
                    left_elbow_rot_angle = 5 * Convert.ToInt32(left_elbow_rot_angle / 5) + 180;

                    left_elbow_angle = Math.Atan2((n.DistanceFormula(l_mdpt_shoulder_wrist, left_shoulder)), (n.DistanceFormula(l_mdpt_shoulder_wrist, left_elbow))) * 180 / (2 * Math.PI);

                    // Sets the bounds for movements for MIMER - for left_elbow_rot_angle
                    if (left_wrist[2] > left_elbow[2] && left_wrist[1] < left_elbow[1])
                    {
                        left_elbow_rot_angle = 0;
                    }
                    else if (left_wrist[2] > left_elbow[2] && left_wrist[1] > left_elbow[1])
                    {
                        left_elbow_rot_angle = 180;
                    }

                    l_elbow_rot_dc = n.dutyCycle(left_elbow_rot_angle);
                    if (left_elbow_rot_angle < 0)
                    {
                        left_elbow_rot_angle = 0;
                    }

                    if (left_elbow_rot_angle > 180)
                    {
                        left_elbow_rot_angle = 180;
                    }

                    // Finds the Elbow Open and Close Angle - uses the midpoint of the shoulder and wrist to find a right angle to take the tangent.
                    l_mdpt_shoulder_wrist = n.Midpoint(left_shoulder, left_wrist);
                    double l_dist_shoulder_mdpt_sw = n.DistanceFormula(left_shoulder, l_mdpt_shoulder_wrist);
                    double l_dist_elbow_mdpt_sw = n.DistanceFormula(left_elbow, l_mdpt_shoulder_wrist);
                    left_elbow_open_close_angle = 2 * Math.Atan2(l_dist_shoulder_mdpt_sw, l_dist_elbow_mdpt_sw) * 180 / Math.PI;                            // r_mdpt_shoulder_wrist = n.Midpoint(right_shoulder, right_wrist); unneeded at the moment
                    l_elbow_open_close_dc = n.dutyCycle(left_elbow_open_close_angle);

                    if (left_elbow_open_close_angle < 0)
                    {
                        left_elbow_open_close_angle = 0;
                    }

                    if (left_elbow_open_close_angle > 180)
                    {
                        left_elbow_open_close_angle = 180;
                    }

                    // Finding Positions and put into the Pololu Controller syntax to be sent
              /*      r_shoulder_up_down_pos = n.PololuPosition(Convert.ToInt32(right_shoulder_up_down_angle),1);
                    r_shoulder_rot_pos = n.PololuPosition(Convert.ToInt32(right_shoulder_rot_angle),2);
                    r_elbow_open_close_pos = n.PololuPosition(Convert.ToInt32(right_elbow_angle),3);
                    r_elbow_rot_pos = n.PololuPosition(Convert.ToInt32(right_elbow_rot_angle),4);

                    l_shoulder_up_down_pos = n.PololuPosition(Convert.ToInt32(left_shoulder_up_down_angle),5);
                    l_shoulder_rot_pos = n.PololuPosition(Convert.ToInt32(left_shoulder_rot_angle),6);
                    l_elbow_open_close_pos = n.PololuPosition(Convert.ToInt32(left_elbow_angle),7);
                    l_elbow_rot_pos = n.PololuPosition(Convert.ToInt32(left_elbow_rot_angle),8);    */

                    // Sending dutyCycles to the PI
                    // Right Arm COMMENTED OUT FOR SPEED TESTING /

                    //r_shoulder_up_down_dc = Math.Abs(r_shoulder_up_down_dc * 10);
                    //Byte r_shoulder_up_down_dc_b = Convert.ToByte(r_shoulder_up_down_dc);
                    //r_shoulder_rot_dc = Math.Abs(r_shoulder_rot_dc * 10);
                    //Byte r_shoulder_rot_dc_b = Convert.ToByte(r_shoulder_rot_dc);
                    //r_elbow_open_close_dc = Math.Abs(r_elbow_open_close_dc * 10);
                    //Byte r_elbow_open_close_dc_b = Convert.ToByte(r_elbow_open_close_dc);
                    //// r_elbow_open_close_dc = Convert.ToInt32(r_elbow_open_close_dc);
                    //r_elbow_rot_dc = Math.Abs(r_elbow_rot_dc * 10);
                    //Byte r_elbow_rot_dc_b = Convert.ToByte(r_elbow_rot_dc);

                    // Left Arm COMMENTED OUT FOR SPEED TESTING

                    //l_shoulder_up_down_dc = Math.Abs(l_shoulder_up_down_dc * 10);
                    //Byte l_shoulder_up_down_dc_b = Convert.ToByte(l_shoulder_up_down_dc);
                    //l_shoulder_rot_dc = Math.Abs(l_shoulder_rot_dc * 10);
                    //Byte l_shoulder_rot_dc_b = Convert.ToByte(l_shoulder_rot_dc);
                    //l_elbow_open_close_dc = Math.Abs(l_elbow_open_close_dc * 10);
                    //Byte l_elbow_open_close_dc_b = Convert.ToByte(l_elbow_open_close_dc);
                    //// l_elbow_open_close_dc = Convert.ToInt32(l_elbow_open_close_dc);
                    //l_elbow_rot_dc = Math.Abs(l_elbow_rot_dc * 10);
                    //Byte l_elbow_rot_dc_b = Convert.ToByte(l_elbow_rot_dc);



                    //byte[] dutycyclelist = { r_shoulder_up_down_dc_b, r_shoulder_rot_dc_b, r_elbow_open_close_dc_b, r_elbow_rot_dc_b };
                /*  byte[] PololuAngleList = { r_shoulder_up_down_pos[0], r_shoulder_up_down_pos[1], r_shoulder_rot_pos[0],
                                                r_shoulder_rot_pos[1], r_elbow_open_close_pos[0], r_elbow_open_close_pos[1], 
                                                r_elbow_rot_pos[0], r_elbow_rot_pos[1],l_shoulder_up_down_pos[0], l_shoulder_up_down_pos[1],
                                                l_shoulder_rot_pos[0], l_shoulder_rot_pos[1], l_elbow_open_close_pos[0], l_elbow_open_close_pos[1],
                                                l_elbow_rot_pos[0], l_elbow_rot_pos[1] };*/
                    for (int k = 0; k < 100; k++)
                    {

                        Stream stm = tcpclnt.GetStream();
                        byte[] anglelist = { Convert.ToByte(right_shoulder_up_down_angle), Convert.ToByte(right_shoulder_rot_angle), Convert.ToByte(right_elbow_rot_angle), Convert.ToByte(right_elbow_open_close_angle), Convert.ToByte(left_shoulder_up_down_angle), Convert.ToByte(left_shoulder_rot_angle), Convert.ToByte(left_elbow_rot_angle), Convert.ToByte(left_elbow_open_close_angle) };

                        //  byte[] dutycyclelist = { Convert.ToByte(100), Convert.ToByte(99), Convert.ToByte(33), Convert.ToByte(56) };

                        stm.Write(anglelist, 0, anglelist.Length);
                    }
                    tcpclnt.Close();  

                        // Joint headJoint = skeleton.Joints[JointType.Head];
                        Joint hipCenter = skeleton.Joints[JointType.HipCenter]; //could probably remove this joint, not sure if needed or not

                        if (headJoint.TrackingState != JointTrackingState.NotTracked)
                        {
                            //SkeletonPoint headPosition = headJoint.Position;

                            message = string.Format("R_Up_Down:{0:0.0} R_S_Rot:{1:0.0}", 
                            //   "A",
                            //   "b",
                            //   "c",
                            right_shoulder_up_down_angle,
                                right_shoulder_rot_angle
                                );
                            //displays the XYZ of the head coordinate, this is for testing

                            //if (headJoint.TrackingState == JointTrackingState.Inferred)
                            //{
                            //    message = message + " I";
                            //}

                            Mirroring nn = new Mirroring();
                            // Spine
                            //addLine(skeleton.Joints[JointType.Head], skeleton.Joints[JointType.ShoulderCenter]); 
                            //addLine(skeleton.Joints[JointType.ShoulderCenter], skeleton.Joints[JointType.Spine]);



                            //// Left arm
                            //addLine(skeleton.Joints[JointType.ShoulderCenter], skeleton.Joints[JointType.ShoulderLeft]);
                            //addLine(skeleton.Joints[JointType.ShoulderLeft], skeleton.Joints[JointType.ElbowLeft]);
                            //addLine(skeleton.Joints[JointType.ElbowLeft], skeleton.Joints[JointType.WristLeft]);
                            //addLine(skeleton.Joints[JointType.WristLeft], skeleton.Joints[JointType.HandLeft]);

                            //// Right arm
                            //addLine(skeleton.Joints[JointType.ShoulderCenter], skeleton.Joints[JointType.ShoulderRight]);
                            //addLine(skeleton.Joints[JointType.ShoulderRight], skeleton.Joints[JointType.ElbowRight]);
                            //addLine(skeleton.Joints[JointType.ElbowRight], skeleton.Joints[JointType.WristRight]);
                            //addLine(skeleton.Joints[JointType.WristRight], skeleton.Joints[JointType.HandRight]);
                        }
                    }

                }

                StatusTextBlock.Text = message;
                QualityTextBlock.Text = qualityMessage;
        }

        private void image1_ImageFailed(object sender, ExceptionRoutedEventArgs e)
        {

        }
    }
}

