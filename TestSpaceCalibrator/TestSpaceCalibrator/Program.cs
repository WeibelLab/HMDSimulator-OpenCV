using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters;
using Microsoft.VisualBasic.FileIO;

namespace TestSpaceCalibrator
{
    struct Sample
    {
        public long timeStamp;
        public Vector3 position;
        public Quaternion rotation;
    }

    class Program
    {
        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int CreateCalibrator();

        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int AddSamples(int handle, float[] refMatrix4x4, float[] targetMatrix4x4);

        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int GetSamples(int handle, int index, int isRef, float[] trans, float[] rot);

        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int ClearSamples(int handle);

        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int PerformCalibration(int handle);

        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int GetRotationQuat(int handle, float[] rot);

        [DllImport("HMDSimulator-OpenCV-Integration.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int GetTranslationVec(int handle, float[] trans);

        static List<Sample> LoadCSV(string path)
        {
            List<Sample> result = new List<Sample>();
            using (TextFieldParser parser = new TextFieldParser(path))
            {
                parser.TextFieldType = FieldType.Delimited;
                parser.SetDelimiters(",");

                if (!parser.EndOfData)
                {
                    parser.ReadLine();
                }

                while (!parser.EndOfData)
                {
                    //Processing row
                    string[] fields = parser.ReadFields();
                    Sample curr = new Sample();
                    curr.timeStamp = Int64.Parse(fields[0]);
                    curr.position = new Vector3(float.Parse(fields[1]), float.Parse(fields[2]), float.Parse(fields[3]));
                    curr.rotation = new Quaternion(float.Parse(fields[4]), float.Parse(fields[5]), float.Parse(fields[6]), float.Parse(fields[7]));
                    result.Add(curr);
                }
            }

            return result;
        }

        static float[] copyMatrixToArray(Matrix4x4 mat)
        {
            float[] result = new float[16];
            result[0] = mat.M11;
            result[1] = mat.M12;
            result[2] = mat.M13;
            result[3] = mat.M14;
            result[4] = mat.M21;
            result[5] = mat.M22;
            result[6] = mat.M23;
            result[7] = mat.M24;
            result[8] = mat.M31;
            result[9] = mat.M32;
            result[10] = mat.M33;
            result[11] = mat.M34;
            result[12] = mat.M41;
            result[13] = mat.M42;
            result[14] = mat.M43;
            result[15] = mat.M44;

            return result;
        }

        static void Main(string[] args)
        {

            // Load data
            //string workingDirectory = Environment.CurrentDirectory;
            //string projectDirectory = Directory.GetParent(workingDirectory).Parent.Parent.Parent.FullName;
            string refCSV = @"./PosXNegYNegZRotXNegRotYNegRotZ-TiaraTracker.csv";
            //@"./PosXNegYNegZRotXNegRotYNegRotZ-TiaraTracker.csv";//@"./HeadsetTranslationAllAxis-TiaraTracker.csv";
            string targetCSV = @"./HeadsetGround-headset.csv ";
            string truthCSV = @"./HeadsetGround-TiaraTracker.csv";
            var refList = LoadCSV(refCSV);
            var targetList = LoadCSV(targetCSV);
            var truthList = LoadCSV(truthCSV);

            // Start calibrate
            int handle = CreateCalibrator();

            // Setup output array
            float[] outRot = new float[4];
            float[] outTrans = new float[3];

            // Use first 1800
            for (int i = 0; i < 900; i++)
            {
                if (i % 2 != 0)
                {
                    //continue;
                }
                var refSample = refList[i];
                var targetSample = targetList[i];

                // create matrix
                Matrix4x4 refRot = Matrix4x4.CreateFromQuaternion(refSample.rotation);
                Matrix4x4 refTrans = Matrix4x4.CreateTranslation(refSample.position);
                Matrix4x4 refMatrix = refRot * refTrans;

                // create matrix
                Matrix4x4 targetRot = Matrix4x4.CreateFromQuaternion(targetSample.rotation);
                Matrix4x4 targetTrans = Matrix4x4.CreateTranslation(targetSample.position);
                Matrix4x4 targetMatrix = targetRot * targetTrans;

                // 
                float[] refArray = copyMatrixToArray(refMatrix);
                float[] targetArray = copyMatrixToArray(targetMatrix);


                int count = AddSamples(handle, refArray, targetArray);

                // 
                if (i == 900 - 1)
                {
                    int result = PerformCalibration(handle);
                    if (result > 0)
                    {
                        result = GetTranslationVec(handle, outTrans);
                        if (result < 0)
                        {
                            Console.Write("Get Translation Error");
                            break;
                        }
                        result = GetRotationQuat(handle, outRot);
                        if (result < 0)
                        {
                            Console.Write("Get rotation Error");
                        }
                    }
                    else
                    {
                        Console.Write("Calibration Error");
                        break;
                    }

                    Vector3 outTransVecDebug = new Vector3(outTrans[0], outTrans[1], outTrans[2]);
                    Quaternion outRotQuatDebug = new Quaternion(outRot[0], outRot[1], outRot[2], outRot[3]);


                    Console.Write(count);
                    Console.WriteLine();
                    Console.Write(outRotQuatDebug.ToString());
                    Console.WriteLine();
                    Console.Write(outTransVecDebug.ToString());
                    Console.WriteLine();
                }

                if (count > 1800)
                {
                    int result = PerformCalibration(handle);
                    if (result > 0)
                    {
                        result = GetTranslationVec(handle, outTrans);
                        if (result < 0)
                        {
                            Console.Write("Get Translation Error");
                            break;
                        }
                        result = GetRotationQuat(handle, outRot);
                        if (result < 0)
                        {
                            Console.Write("Get rotation Error");
                        } 
                        break;
                    }
                    else
                    {
                        Console.Write("Calibration Error");
                        break;
                    }
                }
            }

            // Check result
            Vector3 outTransVec = new Vector3(outTrans[0], outTrans[1], outTrans[2]);
            Quaternion outRotQuat = new Quaternion(outRot[0], outRot[1], outRot[2], outRot[3]);

            Console.Write(outRotQuat.ToString());
            Console.WriteLine();
            Console.Write(outTransVec.ToString());

            // Check truth
            Console.Write("\n\nCheck truth");

            Matrix4x4 resRot = Matrix4x4.CreateFromQuaternion(outRotQuat);
            Matrix4x4 resTrans = Matrix4x4.CreateTranslation(outTransVec);
            Matrix4x4 resMatrix = resRot * resTrans;

            var outRotQuatInv = Quaternion.Inverse(outRotQuat);

            for (int i = 0; i < 900; i++)
            {
                if (i % 100 == 0)
                {
                    var refSample = refList[i];
                    var truthSample = truthList[i];
                    var rotatedVec = refSample.position; //

                    var rotatedQuat = outRotQuatInv * refSample.rotation;


                    rotatedVec.X -= outTransVec.X;//outTransVec.X; 4.096452f;
                    rotatedVec.Y -= outTransVec.Y;//outTransVec.Y; -7.150463f; 
                    rotatedVec.Z -= outTransVec.Z;//outTransVec.Z; 0.432592f; 

                    rotatedVec = Vector3.Transform(rotatedVec, outRotQuatInv);

                    Console.WriteLine();
                    Console.Write("ref:");
                    Console.Write(rotatedVec.ToString());
                    Console.WriteLine();
                    Console.Write("truth:");
                    Console.Write(truthSample.position.ToString());
                    Console.WriteLine();
                    Console.Write("diff:");
                    Console.Write((rotatedVec - truthSample.position).ToString());

                    Console.WriteLine();
                }
            }
        }
    }
}
