using System;
using System.Threading;
using Microsoft.SPOT;
//using Microsoft.SPOT.Hardware;
using Samraksh.eMote;
using Samraksh.eMote.NonVolatileMemory;
using Samraksh.eMote.DotNow;
using Samraksh.eMote.DSP;
//using Samraksh.SPOT.Hardware;
//using Samraksh.SPOT.DSP;
//using Samraksh.SPOT.NonVolatileMemory;
using Samraksh_eMote_Net_Routing;
using System.Collections;


// relation between real length and the big int (VALUE) in the program:
// length = 0.0258 * VALUE/25736 = VALUE * 1.0025e-06 ~= VALUE *1e-6;
enum PI
{
    HALF = 6434,
    FULL = 12868,   //3.1415*4096 = 12868  4096pi
    NEG = -12868,
    TWO = 25736,
}


namespace HumanCarClassifier
{
    public class HumanCarClassifier
    {
        //public static bool waiting = true;
        public static int c = 0;

        public const int fftWindowSize = 256;
        public const int step = 64;   // in fact it is fftWindowSize - overlap, which is the proceed points
        public const int sampleTime = 3906;  // 1/256=0.003906

        public static byte[] packet = new byte[6];
        public const byte nodeID = 0x01;

        public static ushort[] sampleBuffer1 = new ushort[step];
        public static ushort[] sampleBuffer2 = new ushort[step];

        

        public static BufferStorage channelIBuffer;
        public static BufferStorage channelQBuffer;

        public static short[] fftInput = new short[2 * fftWindowSize];
        public static short[] fftOutput = new short[2 * fftWindowSize];

        public static bool stopExperimentFlag = false;

        /*
        public static short [] hammingWindow= new short[256]{
            20, 20, 20, 20, 21, 21, 21, 22, 22, 23, 24, 24, 25, 26, 27, 28, 29, 30, 31, 33, 34, 35, 37, 38, 40, 42, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 66, 68, 70, 73, 75, 78, 80, 83, 85, 88, 91, 93, 96, 99, 101, 104, 107, 110, 113, 115, 118, 121, 124, 127, 130, 133, 136, 138, 141, 144, 147, 150, 153, 156, 159, 162, 164, 167, 170, 173, 176, 178, 181, 184, 186, 189, 192, 194, 197, 199, 202, 204, 206, 209, 211, 213, 215, 218, 220, 222, 224, 226, 228, 229, 231, 233, 235, 236, 238, 239, 241, 242, 243, 245, 246, 247, 248, 249, 250, 251, 252, 252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253, 252, 252, 251, 250, 249, 248, 247, 246, 245, 243, 242, 241, 239, 238, 236, 235, 233, 231, 229, 228, 226, 224, 222, 220, 218, 215, 213, 211, 209, 206, 204, 202, 199, 197, 194, 192, 189, 186, 184, 181, 178, 176, 173, 170, 167, 164, 162, 159, 156, 153, 150, 147, 144, 141, 138, 136, 133, 130, 127, 124, 121, 118, 115, 113, 110, 107, 104, 101, 99, 96, 93, 91, 88, 85, 83, 80, 78, 75, 73, 70, 68, 66, 63, 61, 59, 57, 55, 53, 51, 49, 47, 45, 43, 42, 40, 38, 37, 35, 34, 33, 31, 30, 29, 28, 27, 26, 25, 24, 24, 23, 22, 22, 21, 21, 21, 20, 20, 20, 20
        };*/

        public static int meanI = 2047; // 2837;    // dummy test change
        public static int meanQ = 1924; // 2852;
            

        ////////////////// hardware ////////////////
        public static Microsoft.SPOT.Hardware.OutputPort callbackTime = new Microsoft.SPOT.Hardware.OutputPort(Pins.GPIO_J12_PIN3, false);
        public static Transforms transforms = new Transforms();
        public static Microsoft.SPOT.Hardware.InterruptPort stopExperiment = new Microsoft.SPOT.Hardware.InterruptPort(Pins.GPIO_J11_PIN7, false, Microsoft.SPOT.Hardware.Port.ResistorMode.Disabled, Microsoft.SPOT.Hardware.Port.InterruptMode.InterruptEdgeLow);
        public static EmoteLCD lcd;
        public static Samraksh.eMote.NonVolatileMemory.DataStore dStore;
        public static AdcCallBack adcCallbackPtr;

        
        ////////////////// Timer ////////////////////
        public static Timer timer;



        ////////////////// DataStore related ////////////////
        public static Samraksh.eMote.NonVolatileMemory.DataReference[] dataRefArray;
        public const int nDataRenference = 3;
        ///////////////////////////////////////////





        public static class dState{    // maintained and updated every step
            public static int[] phase;
            public static int[] maxPhaseInStepArr;
            public static int[] minPhaseInStepArr;
            public static int[] diffPhaseInWindowArr;
            public static double[] veloInWindowArr;
            public static double[] accInWindowArr;

            public const int nWindowMax = 40;  //10sec //may be put out of the class later.
            public static int validWindowIndex;   // only starts increasing when displacement is detected
            public static int allWindowIndex;         // starts increasing immediately when the program starts
            public static int startWindowIndex;  // control start of validWindow
            public static int stopWindowIndex;   // control end of validWindow
            public static int stopDisplayIndex; //control the LCD display

            public static int IsumInStep;
            public static int QsumInStep;
            public static int nStepForMeanIQ;

            public static int maxPhaseInWindow;
            public static int minPhaseInWindow;

            public static int diffPhaseInWindow;  //max in window - min in window
            public static int maxPhaseInStep;
            public static int minPhaseInStep;
            public static int phaseArrDiffofTwoEnds;           //last in window - first in window
            public static int cumPhaseChange;
            public static int thr_cumPhaseChange;
            public static int thr_phaseChangeInOneStep;
            public static int phaseChangeInOneStep;
            public static int nStepInStateOne;   // number of steps that is in triggering displacement detection.
            public static int thr_phaseArrDiffofTwoEnds;
            
            
            public static int validFlag;  
            // can be 0, 1, 2. 
            //0 means no movement at all, 1 means trigger process starts and feature calculation starts, 2 means trigger process finishes and feature calculation continues;
            
            public static int validFlag_last;

            public static int sum_phase_diff;

            public static double veloInWindow;

            public static double accInWindow;
            
            public static void init(){
                phase = new int[128];
                maxPhaseInStepArr = new int[fftWindowSize / step];
                minPhaseInStepArr = new int[fftWindowSize / step];
                diffPhaseInWindowArr = new int[nWindowMax];  //nWindow-3
                veloInWindowArr = new double[nWindowMax];  //nWindow-3
                accInWindowArr = new double[nWindowMax];  //nWindow-3

                clearMaxMinPhaseInWindow();
                clearMaxMinPhaseInStep();

                allWindowIndex = 0;
                validWindowIndex = 0;
                startWindowIndex = 0;
                stopWindowIndex = 0;
                stopDisplayIndex = -1;

                // 1000000 corresponding to 1m
                thr_phaseArrDiffofTwoEnds = 100000; // 0.5 second go 0.1m, the speed is 0.2 m/s, in a 0.5s window
                thr_cumPhaseChange = 500000; // 3 seconds (12steps), go 1m, which is 0.33 m/s
                thr_phaseChangeInOneStep = 80000;  // 1/4s, change 1/12 m, which is 0.33 m/s
                phaseChangeInOneStep = 0;
                validFlag = 0;
                validFlag_last = 0;

                IsumInStep = 0;
                QsumInStep = 0;
            }

            public static void clearMaxMinPhaseInWindow()
            {
                maxPhaseInWindow = int.MinValue;
                minPhaseInWindow = int.MaxValue;
            }

            public static void clearMaxMinPhaseInStep()
            {
                maxPhaseInStep = int.MinValue;
                minPhaseInStep = int.MaxValue;
            }

            public static void updateMaxMinPhaseInStepArr()
            {
                Array.Copy(maxPhaseInStepArr, 1, maxPhaseInStepArr, 0, maxPhaseInStepArr.Length-1);
                Array.Copy(minPhaseInStepArr, 1, minPhaseInStepArr, 0, minPhaseInStepArr.Length-1);
                maxPhaseInStepArr[maxPhaseInStepArr.Length-1] = maxPhaseInStep;
                minPhaseInStepArr[minPhaseInStepArr.Length-1] = minPhaseInStep;
            }

            public static void rotatePhase()
            {
                Array.Copy(phase, step, phase, 0, 128 - step);
            }

        }   

        public class BufferStorage
        {
            public int fftWindowSize;
            public int step;
            public ushort[] buffer;

            public Object bufferLock = new object();

            public bool bufferfull = false;

            public BufferStorage(int fftWindowSize, int step)  //, uint fftWindowSize
            {
                this.step = step;
                this.fftWindowSize = fftWindowSize;

                buffer = new ushort[fftWindowSize];
                //buffer = new ushort[step];
            }

            public void CopyFrom(ushort[] arr)
            {
                lock (bufferLock)
                {
                    arr.CopyTo(buffer, fftWindowSize - step);//(int)(fftWindowSize - overlap) //可以用Array.Copy替换
                    bufferfull = true;
                }
            }

            public bool IsFull()
            {
                lock (bufferLock)
                {
                    return bufferfull;
                }
            }
        } 
                
        public static class Phase   // only work in one step
        {
            public static int wPhase;
            public static int uwPhase;
            public static int wPhase_prev;
            public static int uwPhase_prev;
            public static int phase_diff;
            public static int newPhase;

            public static int I_prev;
            public static int Q_prev;
            public static int CumCuts;

            //public static short[] arcTan;

            public static int[] uwPhaseArr;

            public static int[] tanArr;
            public static int[] arcTanArr;

            public static ushort[] readBuffer;


            public static void init()
            {
                wPhase = 0;
                uwPhase = 0;
                wPhase_prev = 0;
                uwPhase_prev = 0;

                uwPhaseArr = new int[step];
                tanArr = new int[step];
                arcTanArr = new int[step];

                //// DataStore related
                
                readBuffer = new ushort[1];


                CumCuts = 0;
            }

            public static int findArcTan(int small, int big)
            {
                if (big == 0) return 0;
                int tmp = small * 4096 / big;


                dataRefArray[0].Read(readBuffer, tmp, readBuffer.Length);

                return readBuffer[0];
                //return arcTan[tmp];
            }
            
            /*
            public static void unwrap(short[] Is, short[] Qs)
            {

                for (int i = 0; i < Is.Length; i++)
                {
                    tanArr[i] = System.Math.Min(System.Math.Abs(Is[i]), System.Math.Abs(Qs[i])) * 4096 / System.Math.Max(System.Math.Abs(Is[i]), System.Math.Abs(Qs[i]));
                }

                for (int i = 0; i < tanArr.Length; i++)
                {
                    data.Read(readBuffer, tanArr[i], readBuffer.Length);
                    arcTanArr[i] = readBuffer[0];
                }

                for (int i = 0; i < Is.Length; i++)
                {
                    short I = Is[i];
                    short Q = Qs[i];
                    if (I >= 0 && Q >= 0)
                    {			//1st Quadrant: arg = atan(imag/real)
                        if (Q <= I) 					// Q/I is in {0,1}
                            wPhase = arcTanArr[i];
                        else
                            wPhase = (int)PI.HALF - arcTanArr[i];	// atan(x) = pi/2 - atan(1/x) for x > 0		
                    }
                    else if (I < 0 && Q >= 0)
                    {		//2nd quadrant: arg = pi - atan(abs(imag/real)
                        if (Q <= -I)
                            wPhase = (int)PI.FULL - arcTanArr[i];
                        else
                            wPhase = (int)PI.HALF + arcTanArr[i];  // pi - (pi/2 - atan(1/x))
                    }
                    else if (I < 0 && Q < 0)
                    {			// 3rd quadrant: arg = -pi + atan(b/a)
                        if (-Q <= -I)
                            wPhase = - (int)PI.FULL + arcTanArr[i];
                        else
                            wPhase = - (int)PI.HALF - arcTanArr[i];	// -pi + pi/2 - atan(1/x)
                    }
                    else if (I >= 0 && Q < 0)
                    {							//4th quadrant: arg = - atan(b/a)
                        if (-Q <= I)
                            wPhase = - arcTanArr[i];
                        else
                            wPhase = - (int)PI.HALF + arcTanArr[i];
                    }

                    phase_diff = wPhase - wPhase_prev;
                    if (phase_diff < (int)PI.NEG)
                        phase_diff += (int)PI.TWO;
                    else if (phase_diff > (int)PI.FULL)
                        phase_diff -= (int)PI.TWO;
                    uwPhase = uwPhase_prev + phase_diff;

                    uwPhaseArr[i] = uwPhase;           ////////////////////// key line

                    wPhase_prev = wPhase;
                    uwPhase_prev = uwPhase;
                }
            }
             * */

            /*
            public static int unwrap(short I, short Q)
            {
                if ((I_prev * Q - I * Q_prev < 0) && (Q > 0) && (Q_prev < 0))
                    CumCuts += 1;
                else if ((I_prev * Q - I * Q_prev > 0) && (Q < 0) && (Q_prev> 0))
                    CumCuts -= 1;

                I_prev = I;
                Q_prev = Q;

                uwPhase = CumCuts;
                return CumCuts;
            }*/

            
            public static int unwrap(short I, short Q)
            {
                newPhase = 0;

                if (I >= 0 && Q >= 0)
                {			//1st Quadrant: arg = atan(imag/real)
                    if (Q <= I) 					// Q/I is in {0,1}
                        newPhase = findArcTan(Q, I);
                    else
                        newPhase = (int)PI.HALF - findArcTan(I, Q);	// atan(x) = pi/2 - atan(1/x) for x > 0		
                }
                else if (I < 0 && Q >= 0)
                {		//2nd quadrant: arg = pi - atan(abs(imag/real)
                    if (Q <= System.Math.Abs(I))
                        newPhase = (int)PI.FULL - findArcTan(Q, System.Math.Abs(I));
					else
                        newPhase = (int)PI.HALF + findArcTan(System.Math.Abs(I), Q);  // pi - (pi/2 - atan(1/x))
                }

                else if (I < 0 && Q < 0)
                {			// 3rd quadrant: arg = -pi + atan(b/a)
                    if (System.Math.Abs(Q) <= System.Math.Abs(I))
                        newPhase = 0 - (int)PI.FULL + findArcTan(System.Math.Abs(Q), System.Math.Abs(I));
                    else
                        newPhase = 0 - (int)PI.HALF - findArcTan(System.Math.Abs(I), System.Math.Abs(Q));	// -pi + pi/2 - atan(1/x)
                }

                else if (I >= 0 && Q < 0)
                {							//4th quadrant: arg = - atan(b/a)
                    if (System.Math.Abs(Q) <= I)
                        newPhase = 0 - findArcTan(System.Math.Abs(Q), I);
                    else
                        newPhase = 0 - (int)PI.HALF + findArcTan(I, System.Math.Abs(Q));
                }

                wPhase = newPhase;

                phase_diff = wPhase - wPhase_prev;
                if (phase_diff < (int)PI.NEG)
                    phase_diff += (int)PI.TWO;
                else if (phase_diff > (int)PI.FULL)
                    phase_diff -= (int)PI.TWO;
                uwPhase = uwPhase_prev + phase_diff;

                if (phase_diff >= 0)
                    dState.sum_phase_diff += phase_diff;
                else
                    dState.sum_phase_diff -= phase_diff;

                wPhase_prev = wPhase;
                uwPhase_prev = uwPhase;

                return uwPhase;
            }
        }


        ////////////////////////////////////////////////

        public static class Polyfit
        {
            public static int p, rs;
            public static double[][] m;
            public static double[] mpc;
            public static double[] polyparams = new double[p];   // order is: polyparams[0] + polyparams[1]*x +polyparams[2]*x^2+ ...  opposite to the Matlab order

            public static long[] yTimesPowerx;
            public static double[] powerx_sum;
            public static int[] TauTimes1000;

            public static void init()
            {
                int degree = 3;
                p = degree + 1;
                rs = 2 * p - 1;

                TauTimes1000 = new int[128]{
                    -500, -492, -484, -476, -468, -460, -452, -444, -437, -429, -421, -413, -405, -397, -389, -381, -374, -366, -358, -350, -342, -334, -326, -318, -311, -303, -295, -287, -279, -271, -263, -255, -248, -240, -232, -224, -216, -208, -200, -192, -185, -177, -169, -161, -153, -145, -137, -129, -122, -114, -106, -98, -90, -82, -74, -66, -59, -51, -43, -35, -27, -19, -11, -3, 3, 11, 19, 27, 35, 43, 51, 59, 66, 74, 82, 90, 98, 106, 114, 122, 129, 137, 145, 153, 161, 169, 177, 185, 192, 200, 208, 216, 224, 232, 240, 248, 255, 263, 271, 279, 287, 295, 303, 311, 318, 326, 334, 342, 350, 358, 366, 374, 381, 389, 397, 405, 413, 421, 429, 437, 444, 452, 460, 468, 476, 484, 492, 500
                };

                powerx_sum = new double[7] { 128f, 0f, 10.8065f, 0f, 1.6440f, 0f, 0.2978f }; //7=rs


                polyparams = new double[p];
                m = new double[p][];
                for (int i = 0; i < p; i++)
                {
                    m[i] = new double[p + 1];
                }
                mpc = new double[rs];

                yTimesPowerx = new long[p];
            }

            public static void addPoint(int[] y)   ///y must be length 128
            {
                // process precalculation array
                /*
                int powerx = x;
                for (int i = 1; i < rs; i++)
                {
                    powerxArr[7][i] += powerx;   //mpc[i]
                    powerx *= x;
                }*/

                //clear it before summing up
                for (int j = 0; j < p; j++)
                    yTimesPowerx[j] = 0;

                for (int i = 0; i < y.Length; i++)
                {
                    long tmp = y[i];
                    long x = TauTimes1000[i];

                    for (int j = 0; j < p; j++)
                    {
                        yTimesPowerx[j] += tmp;   //m[i][p]
                        tmp *= x;                    
                    }
                }
            }

            public static void getBestFit()
            {
                Array.Copy(powerx_sum, mpc, rs);
                for (int i = 0; i < p; i++)
                {
                    m[i][p] = yTimesPowerx[i] / System.Math.Pow(1000, i);
                }

                /*
                int[] mpcClone = (int[])mpc.Clone();
                float[][] mClone = new float[m.Length][];
                for (int i = 0; i < m.Length; i++)
                {
                    mClone[i] = new float[m[0].Length];
                    for (int j = 0; j < m[0].Length; j++)
                        mClone[i][j] = (float)m[i][j];
                }*/

                

                // populate square matrix section
                for (int r = 0; r < p; r++)
                {
                    for (int c = 0; c < p; c++)
                    {
                        m[r][c] = mpc[r + c];
                    }
                }
                gj_echelonize(m);

                for (int i = 0; i < p; i++)
                {
                    polyparams[i] = m[i][p];
                }
            }

            

            private static void gj_echelonize(double[][] A)
            {
                int n = A.Length;
                int m = A[0].Length;
                int i = 0;
                int j = 0;
                while (i < n && j < m)
                {
                    // look for a non-zero entry in col j at or below row i
                    int k = i;
                    while (k < n && A[k][j] == 0)
                    {
                        k++;
                    }
                    // if such an entry is found at row k
                    if (k < n)
                    {
                        // if k is not i, then swap row i with row k
                        if (k != i)
                        {
                            gj_swap(A, i, j);
                        }
                        // if A[i][j] is not 1, then divide row i by A[i][j]
                        if (A[i][j] != 1)
                        {
                            gj_divide(A, i, j, m);
                        }
                        // eliminate all other non-zero entries from col j by
                        // subtracting from each
                        // row (other than i) an appropriate multiple of row i
                        gj_eliminate(A, i, j, n, m);
                        i++;
                    }
                    j++;
                }
            }

            private static void gj_eliminate(double[][] A, int i, int j, int n, int m)
            {
                for (int k = 0; k < n; k++)
                {
                    if (k != i && A[k][j] != 0)
                    {
                        for (int q = j + 1; q < m; q++)
                        {
                            A[k][q] -= A[k][j] * A[i][q];
                        }
                        A[k][j] = 0;
                    }
                }
            }

            private static void gj_divide(double[][] A, int i, int j, int m)
            {
                for (int q = j + 1; q < m; q++)
                {
                    A[i][q] /= A[i][j];
                }
                A[i][j] = 1;
            }

            private static void gj_swap(double[][] A, int i, int j)
            {
                double[] temp;
                temp = A[i];
                A[i] = A[j];
                A[j] = temp;
            }


            public static double polyfit1(int[] arr)
            {

                int sumx = 8128;  //matlab sum(0:127)
                //int sumx2 = 690880;  // matlab sum((0:127).^2)
                long sumy = 0;
                long sumxy = 0;
                    
                for (int i = 0; i < 128; i++)
                {
                    //sumx += i;
                    sumy += arr[i];
                    sumxy += i * arr[i];
                    //sumx2 += i*i;
                }

                // (N * sumxy - sumx * sumy)/(N * sumx2 - sumx * sumx)
                double slope = (5.7224e-06 * sumxy - 4.4706e-08 * sumx * sumy);   // 22368256 = 128 * sumx2 - sumx * sumx;   4.4706e-08 = 1/22368256  , 4.4706e-08 * 128 = 5.7224e-06
                if (slope < 0) slope = -slope;
                return slope;
            }

        }



        public static class Sorter
        {
            public static void Sort(int[] a)
            {
                QuickSort(a, 0, a.Length - 1);
            }
            public static void QuickSort(int[] a, int start, int end)
            {   
                if (start >= end) return;
                int q = partition(a, start, end);
                QuickSort(a, start, q - 1);
                QuickSort(a, q + 1, end);

            }
            public static int partition(int[] a, int start, int end)
            {
                int pivotIndex = start;//start+(int)(Math.random()*(end-start));
                int pivot = a[pivotIndex];
                swap(a, pivotIndex, end);
                int storeIndex = start;
                for (int i = start; i <= end - 1; i++)
                {
                    if (a[i] <= pivot)
                    {
                        swap(a, i, storeIndex++);
                    }
                }
                swap(a, storeIndex, end);
                return storeIndex;
            }

            public static void swap(int[] a, int i, int j)
            {
                int tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
        }

        public static class Spectrogram
        {
            //public static int[] freq;
            public static int[] numHitBinsArr;
            public static int[] momentArr;
            //public static double[] thr_sqr_Csharp;
            public static double thr_sqr_Csharp;
            public static int numHitBins;
            public static int moment;
            public static int moment_sum;
            public static int numHitBins_sum;
            public static int maxFreqInWindow;
            public static int maxFreqThroughWindows;
            public static int MaxRunLenInWindow;
            public static int maxRunLenThroughWindows;

            public static void init()
            {
                numHitBins = 0;
                moment = 0;
                numHitBins_sum = 0;
                moment_sum = 0;
                maxFreqInWindow = -1;
                maxFreqThroughWindows = -1;
                MaxRunLenInWindow = -1;
                maxRunLenThroughWindows = -1;
                //freq = new int[256] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };
                
                /*
                thr_sqr_Csharp = new double[256] { 
                    5480.1784, 8998.7677, 5408.5591, 2329.2154, 1385.4272, 1078.6128, 772.7511, 634.7342, 620.5225, 495.4794, 455.7357, 415.0497, 352.055, 263.2085, 249.7452, 240.7964, 223.55, 200.6467, 172.5819, 180.1853, 175.9794, 173.9058, 160.327, 196.3925, 229.1717, 181.6934, 153.0085, 140.0323, 133.8748, 131.4838, 133.722, 143.3942, 136.5305, 121.1017, 120.4828, 113.9682, 118.489, 117.3669, 116.7402, 109.0496, 120.5419, 104.5897, 119.2215, 106.5765, 117.0243, 113.4824, 96.7675, 114.8313, 98.1752, 94.5188, 98.5435, 109.156, 102.0201, 97.109, 93.7558, 92.5926, 93.2524, 99.2438, 86.1421, 90.9327, 81.8472, 82.7143, 71.7765, 80.9017, 80.1203, 70.6759, 72.1011, 71.3434, 78.571, 80.1396, 70.8164, 74.2997, 67.2523, 70.1636, 68.4333, 62.7254, 60.4111, 62.385, 62.4304, 61.5449, 63.0649, 63.5722, 64.2501, 60.2693, 55.0806, 58.675, 56.0285, 54.9855, 57.7582, 58.0222, 62.6073, 57.6706, 54.7871, 53.0909, 58.1962, 54.9837, 49.1384, 54.402, 59.266, 52.5896, 54.567, 47.6604, 49.1666, 51.9543, 47.789, 50.8012, 44.9016, 46.6311, 44.5541, 42.515, 46.3958, 44.2272, 47.6144, 43.8715, 44.3965, 47.5344, 50.7221, 47.9257, 46.4042, 48.3674, 47.1753, 44.6416, 45.6488, 40.3675, 45.7243, 41.6986, 42.3297, 43.8115, 37.0944, 43.8115, 42.3297, 41.6986, 45.7243, 40.3675, 45.6488, 44.6416, 47.1753, 48.3674, 46.4042, 47.9257, 50.7221, 47.5344, 44.3965, 43.8715, 47.6144, 44.2272, 46.3958, 42.515, 44.5541, 46.6311, 44.9016, 50.8012, 47.789, 51.9543, 49.1666, 47.6604, 54.567, 52.5896, 59.266, 54.402, 49.1384, 54.9837, 58.1962, 53.0909, 54.7871, 57.6706, 62.6073, 58.0222, 57.7582, 54.9855, 56.0285, 58.675, 55.0806, 60.2693, 64.2501, 63.5722, 63.0649, 61.5449, 62.4304, 62.385, 60.4111, 62.7254, 68.4333, 70.1636, 67.2523, 74.2997, 70.8164, 80.1396, 78.571, 71.3434, 72.1011, 70.6759, 80.1203, 80.9017, 71.7765, 82.7143, 81.8472, 90.9327, 86.1421, 99.2438, 93.2524, 92.5926, 93.7558, 97.109, 102.0201, 109.156, 98.5435, 94.5188, 98.1752, 114.8313, 96.7675, 113.4824, 117.0243, 106.5765, 119.2215, 104.5897, 120.5419, 109.0496, 116.7402, 117.3669, 118.489, 113.9682, 120.4828, 121.1017, 136.5305, 143.3942, 133.722, 131.4838, 133.8748, 140.0323, 153.0085, 181.6934, 229.1717, 196.3925, 160.327, 173.9058, 175.9794, 180.1853, 172.5819, 200.6467, 223.55, 240.7964, 249.7452, 263.2085, 352.055, 415.0497, 455.7357, 495.4794, 620.5225, 634.7342, 772.7511, 1078.6128, 1385.4272, 2329.2154, 5408.5591, 8998.7677
                };*/
                thr_sqr_Csharp = 809.5268;
                
                momentArr = new int[dState.nWindowMax];  //dState.nWindow-3
                numHitBinsArr = new int[dState.nWindowMax];  //dState.nWindow-3
            }
            public static int freq(int i)
            {
                return i < 128 ? i : (256 - i);
            }
            public static double mean(ushort[] arr)
            {
                double res = 0;
                for (int i = 0; i < arr.Length; i++)
                    res += arr[i];
                res /= arr.Length;
                return res;
            }
            public static double mean(int[] arr)
            {
                double res = 0;
                for (int i = 0; i < arr.Length; i++)
                    res += arr[i];
                res /= arr.Length;
                return res;
            }
            public static double var(int[] arr)
            {
                double res = 0;
                double m = mean(arr);

                for (int i = 0; i < arr.Length; i++)
                    res += (arr[i] - m) * (arr[i] - m);
                res /= arr.Length - 1;
                return res;
            }

            public static double std(int[] arr)
            {
                double res = 0;
                double v = var(arr);
                res = System.Math.Sqrt(v);
                return res;
            }
        }

        public static class DecisionFunction
        {

            public const int nSV = 78;
            public static double rho;//-139.95442911188994;
            public static double gamma;  // 0.01
            public static double[][] sv;
            public static double[] weight;
            public static double[] features;
            public static double[] features_normalized;
            public static double[] feature_min;
            public static double[] scalingFactors;

            public static void init(){
                
                //sv[] = new double[18] { };
                //Debug.Print("mem_beforeSV="+Debug.GC(true));

                ////////////////////////////////////////// paste starts
                rho = 2.3227;
                gamma = 0.5;
                sv = new double[nSV][];
                sv[0] = new double[18] { 0.082231, 0.087445, 0.099951, 0.14648, 0.1416, 0.11654, 0.001444, 0, 0, 0.18197, 0.003016, 0.000613, 0.00143, 0.000102, 0.15625, 0, 0.052522, 0 };
                sv[1] = new double[18] { 0.14005, 0.21338, 0.21376, 0.16942, 0.2693, 0.25275, 0.003588, 0.009359, 0.000312, 0.15092, 0.045838, 0.020594, 0.01666, 0.002358, 0.24219, 0, 0.46458, 0 };
                sv[2] = new double[18] { 0.45174, 0.50916, 0.45697, 0.43762, 0.50826, 0.40119, 0.028159, 0.028078, 0.002471, 0.30838, 0.000603, 3e-05, 0.00066, 2.7e-05, 0, 0, 0.02601, 0 };
                sv[3] = new double[18] { 0.30365, 0.2564, 0.23869, 0.3138, 0.30568, 0.25195, 0.20863, 0.25271, 0.063482, 0.38594, 0.010253, 0.000327, 0.008393, 0.000186, 0.33594, 0, 0.099495, 0 };
                sv[4] = new double[18] { 0.049132, 0.1207, 0.14519, 0.10227, 0.18199, 0.15693, 0.020311, 0.014039, 0.001374, 0.32538, 0, 0, 0, 0, 0, 0, 0.090828, 0 };
                sv[5] = new double[18] { 0.34176, 0.26427, 0.058451, 0.31745, 0.28774, 0.07774, 0.13598, 0.24335, 0.041733, 0.22248, 0.12787, 0.032362, 0.049492, 0.003162, 0.19531, 0, 0.10806, 0 };
                sv[6] = new double[18] { 0.12308, 0.1852, 0.18406, 0.12949, 0.24792, 0.19792, 0.002598, 0.028078, 0.000679, 0.061038, 0.072376, 0.004302, 0.018672, 0.000116, 0.1875, 0, 0.20636, 0 };
                sv[7] = new double[18] { 0.23041, 0.2503, 0.21087, 0.32676, 0.28034, 0.22585, 0.06299, 0.046797, 0.00639, 0.48227, 0, 0, 0, 0, 0, 0, 0.10947, 0 };
                sv[8] = new double[18] { 0.3967, 0.10211, 0.10492, 0.39919, 0.15547, 0.10345, 0.2969, 0.24934, 0.087233, 0.60823, 0, 0, 0, 0, 0, 0, 0.31618, 0 };
                sv[9] = new double[18] { 0.29716, 0.10076, 0.10347, 0.19494, 0.14129, 0.080819, 0.36192, 0.29014, 0.1199, 0.65377, 0.003016, 5e-05, 0.003961, 6.4e-05, 0.40625, 0, 0.3428, 0 };
                sv[10] = new double[18] { 0.27808, 0.32503, 0.30775, 0.27344, 0.36857, 0.31129, 0.027221, 0.056157, 0.003681, 0.17123, 0.065139, 0.01496, 0.020385, 0.001025, 0.039063, 0, 0.25343, 0 };
                sv[11] = new double[18] { 0.53422, 0.57715, 0.55984, 0.62068, 0.62472, 0.52157, 0.032486, 0.018719, 0.002303, 0.42462, 0.025332, 0.001625, 0.023182, 0.001809, 0.44531, 0, 0.10142, 0 };
                sv[12] = new double[18] { 0.2711, 0.29869, 0.12542, 0.27326, 0.34039, 0.15831, 0.36109, 0.42469, 0.16781, 0.41734, 0.048854, 0.00116, 0.030962, 0.000339, 0.24219, 0, 0.35403, 0 };
                sv[13] = new double[18] { 0.47316, 0.27119, 0.11872, 0.56046, 0.41194, 0.16667, 0.24744, 0.23399, 0.069656, 0.52318, 0.15259, 0.028708, 0.082891, 0.00793, 0.375, 0, 0.15292, 0 };
                sv[14] = new double[18] { 0.31669, 0.38851, 0.44407, 0.36309, 0.40879, 0.41897, 0.01693, 0, 0.000679, 0.42173, 0, 0, 0, 0, 0, 0, 0.22714, 0 };
                sv[15] = new double[18] { 0.30709, 0.11226, 0.13029, 0.22828, 0.17654, 0.13612, 0.27824, 0.22463, 0.075129, 0.62932, 0, 0, 0, 0, 0, 0, 0.16803, 0 };
                sv[16] = new double[18] { 0.25459, 0.32114, 0.35304, 0.28251, 0.39101, 0.39676, 0.00886, 0.009359, 0.000589, 0.21894, 0.026538, 0.001994, 0.012841, 0.000277, 0.23438, 0, 0.074263, 0 };
                sv[17] = new double[18] { 0.038029, 0.07342, 0.093143, 0.056952, 0.10579, 0.061838, 0.37055, 0.327, 0.13614, 0.58924, 0, 0, 0, 0, 0, 0, 0.24841, 0 };
                sv[18] = new double[18] { 0.063575, 0.099492, 0.097931, 0.14917, 0.14706, 0.10453, 0.11453, 0.20591, 0.031143, 0.21916, 0, 0, 0, 0, 0, 0, 0.16247, 0 };
                sv[19] = new double[18] { 0.34176, 0.26427, 0.058451, 0.31745, 0.28774, 0.07774, 0.13598, 0.24335, 0.041733, 0.22248, 0.12787, 0.032362, 0.049492, 0.003162, 0.19531, 0, 0.10806, 0 };
                sv[20] = new double[18] { 0.35277, 0.40792, 0.44615, 0.40325, 0.46021, 0.47143, 0.073185, 0.093595, 0.011424, 0.31737, 0.23703, 0.024113, 0.16377, 0.008211, 0, 0, 0.048106, 0 };
                sv[21] = new double[18] { 0.1106, 0.10572, 0.090483, 0.14243, 0.12526, 0.10314, 0.17709, 0.32919, 0.068846, 0.21202, 0.2304, 0.028284, 0.05886, 0.001509, 0.1875, 0, 0.053551, 0 };
                sv[22] = new double[18] { 0.38403, 0.15165, 0.075151, 0.35615, 0.15412, 0.077685, 0.24087, 0.31427, 0.087638, 0.3558, 0.056092, 0.004191, 0.072423, 0.005513, 0, 0, 0.15318, 0 };
                sv[23] = new double[18] { 0.21889, 0.03878, 0.048102, 0.23295, 0.11416, 0.021664, 0.073783, 0.17534, 0.018781, 0.13985, 0.008444, 0.000395, 0.003803, 8.7e-05, 0.21875, 0, 0.43434, 0 };
                sv[24] = new double[18] { 0.27808, 0.32503, 0.30775, 0.27344, 0.36857, 0.31129, 0.027221, 0.056157, 0.003681, 0.17123, 0.065139, 0.01496, 0.020385, 0.001025, 0.039063, 0, 0.25343, 0 };
                sv[25] = new double[18] { 0.64608, 0.70441, 0.76192, 0.68739, 0.72919, 0.79995, 0.083846, 0.037438, 0.007296, 0.73417, 0.008444, 0.000862, 0.014098, 0.001848, 0.57813, 0, 0.73152, 0 };
                sv[26] = new double[18] { 0.26649, 0.263, 0.27219, 0.2708, 0.31805, 0.2941, 0.13491, 0.20591, 0.035965, 0.27759, 0.015682, 0.000365, 0.010813, 0.000145, 0.38281, 0, 0.024699, 0 };
                sv[27] = new double[18] { 0.6489, 0.31734, 0.14191, 0.64379, 0.3563, 0.15396, 0.31966, 0.24335, 0.09163, 0.68115, 0, 0, 0, 0, 0, 0, 0.43289, 0 };
                sv[28] = new double[18] { 0.14722, 0.19476, 0.18094, 0.14543, 0.23166, 0.19747, 0.093758, 0.15911, 0.021187, 0.23459, 0.079614, 0.008048, 0.030789, 0.000857, 0, 0, 0.2079, 0 };
                sv[29] = new double[18] { 0.30426, 0.18293, 0.0875, 0.29474, 0.18886, 0.069098, 0.17981, 0.29058, 0.062533, 0.26161, 0.079011, 0.012523, 0.034121, 0.001806, 0.13281, 0, 0.12633, 0 };
                sv[30] = new double[18] { 0.23049, 0.16095, 0.14874, 0.30875, 0.20985, 0.13237, 0.20432, 0.17783, 0.046456, 0.55746, 0, 0, 0, 0, 0, 0, 0.95487, 0 };
                sv[31] = new double[18] { 0.51402, 0.35439, 0.15699, 0.55312, 0.41044, 0.16484, 0.42199, 0.37438, 0.17382, 0.5923, 0.22376, 0.038861, 0.10978, 0.00752, 0.34375, 0, 0.20721, 0 };
                sv[32] = new double[18] { 0.32152, 0.37742, 0.40141, 0.35253, 0.36727, 0.38718, 0.034218, 0.056157, 0.004356, 0.22047, 0.018094, 0.001661, 0.010593, 0.000422, 0.21094, 0, 0.37171, 0 };
                sv[33] = new double[18] { 0.46708, 0.52034, 0.5671, 0.49805, 0.53502, 0.54127, 0.043055, 0, 0.001823, 0.8262, 0, 0, 0, 0, 0, 0, 0.18043, 0 };
                sv[34] = new double[18] { 0.16715, 0.24559, 0.20411, 0.22304, 0.29092, 0.2084, 0.004124, 0.028078, 0.000786, 0.075798, 0.08263, 0.012071, 0.017839, 0.000214, 0.24219, 0, 0.16576, 0 };
                sv[35] = new double[18] { 0.45559, 0.51965, 0.39469, 0.48085, 0.5217, 0.42178, 0.034317, 0.037438, 0.00339, 0.30816, 0.072979, 0.035367, 0.018169, 0.001329, 0.21094, 0, 0.56675, 0 };
                sv[36] = new double[18] { 0.11312, 0, 0, 0.19991, 0, 0, 0.040754, 0.071951, 0.005903, 0.20837, 0, 0, 0, 0, 0, 0, 0.009871, 0 };
                sv[37] = new double[18] { 0.20714, 0.1697, 0.18018, 0.24503, 0.22081, 0.19184, 0.29205, 0.46797, 0.15004, 0.27076, 0.24065, 0.013131, 0.063638, 0.000791, 0.10938, 0, 0.20352, 0 };
                sv[38] = new double[18] { 0.19011, 0.2457, 0.23059, 0.21826, 0.28029, 0.23214, 0.14295, 0.23399, 0.042186, 0.25356, 0.007841, 0.00014, 0.004149, 3e-05, 0.17969, 0, 0.037595, 0 };
                sv[39] = new double[18] { 0.20774, 0.26561, 0.28479, 0.2589, 0.29876, 0.28534, 0.004059, 0.009359, 0.000337, 0.15699, 0.085645, 0.016562, 0.038616, 0.002587, 0.023438, 0, 0.19282, 0 };
                sv[40] = new double[18] { 0.23651, 0.075269, 0.080499, 0.20365, 0.10805, 0.072684, 0.20165, 0.1722, 0.04472, 0.56735, 0, 0, 0, 0, 0, 0, 0.21785, 0 };
                sv[41] = new double[18] { 0.17658, 0.23123, 0.23745, 0.18349, 0.28309, 0.29036, 0.023122, 0.056157, 0.003286, 0.14238, 0.008444, 0.000643, 0.002248, 4.2e-05, 0.17969, 0, 0.16521, 0 };
                sv[42] = new double[18] { 0.16851, 0.096281, 0.099804, 0.17127, 0.16719, 0.095378, 0.1221, 0.13103, 0.02288, 0.41398, 0.004222, 0.000133, 0.002185, 2.6e-05, 0, 0, 0.092473, 0 };
                sv[43] = new double[18] { 0.11451, 0.069835, 0.020209, 0.14206, 0.10761, 0.033165, 0.017591, 0.084235, 0.003775, 0.044797, 0.099517, 0.003577, 0.022114, 0.001191, 0.4375, 0, 0.43549, 0 };
                sv[44] = new double[18] { 0.21777, 0.15499, 0.15647, 0.21868, 0.22661, 0.16972, 0.029296, 0.093595, 0.005655, 0.090864, 0.12726, 0.012518, 0.036212, 0.001735, 0.25781, 0, 0.087581, 0 };
                sv[45] = new double[18] { 0.14932, 0.19232, 0.24161, 0.21693, 0.31729, 0.26486, 0.006453, 0, 0.000219, 0.25952, 0.003619, 0.000836, 0.00176, 0.000148, 0.16406, 0, 0.15749, 0 };
                sv[46] = new double[18] { 0.28278, 0.30032, 0.23503, 0.33424, 0.32868, 0.25823, 0.033, 0.084235, 0.005666, 0.13, 0.13993, 0.019104, 0.040817, 0.002184, 0.19531, 0, 0.06429, 0 };
                sv[47] = new double[18] { 0.23408, 0.14136, 0.10893, 0.21505, 0.1799, 0.13251, 0.23901, 0.20839, 0.061203, 0.5688, 0.00965, 0.000304, 0.004589, 6.2e-05, 0.11719, 0, 0.10686, 0 };
                sv[48] = new double[18] { 0.3845, 0.41277, 0.41614, 0.40197, 0.47867, 0.45951, 0.06985, 0.093595, 0.010986, 0.30016, 0.00965, 0.000488, 0.002703, 3.8e-05, 0, 0, 0.46681, 0 };
                sv[49] = new double[18] { 0.47037, 0.3757, 0.21871, 0.48236, 0.441, 0.26626, 0.24873, 0.31822, 0.091294, 0.36571, 0.18757, 0.021047, 0.15832, 0.0131, 0.35938, 0, 0.3502, 0 };
                sv[50] = new double[18] { 0.11201, 0.10963, 0.063851, 0.1361, 0.17455, 0.082572, 0.011363, 0.056157, 0.002152, 0.05963, 0.001206, 3.9e-05, 0.000157, 1e-06, 0, 0, 0.11413, 0 };
                sv[51] = new double[18] { 0.11648, 0.11434, 0.098588, 0.12335, 0.15666, 0.078831, 0.11108, 0.17257, 0.026195, 0.26698, 0.050663, 0.003697, 0.011615, 0.000216, 0.03125, 0, 0.096817, 0 };
                sv[52] = new double[18] { 0.50379, 0.54468, 0.58798, 0.52675, 0.59113, 0.59835, 0.15127, 0.14975, 0.030763, 0.4659, 0.0193, 0.000922, 0.020731, 0.000793, 0.35938, 0, 0.19712, 0 };
                sv[53] = new double[18] { 0.47272, 0.5359, 0.56205, 0.49228, 0.57113, 0.54679, 0.16719, 0.16847, 0.036996, 0.46414, 0.024125, 0.001159, 0.025681, 0.00101, 0.375, 0, 0.040901, 0 };
                sv[54] = new double[18] { 0.34904, 0.33781, 0.14364, 0.34817, 0.38645, 0.16049, 0.15793, 0.20591, 0.041413, 0.34361, 0, 0, 0, 0, 0, 0, 0.005225, 0 };
                sv[55] = new double[18] { 0.4758, 0.52312, 0.55029, 0.47597, 0.54795, 0.57068, 0.0908, 0.10295, 0.014722, 0.37211, 0.07117, 0.005963, 0.057241, 0.002672, 0.32813, 0, 0.13345, 0 };
                sv[56] = new double[18] { 0.20685, 0.28014, 0.3155, 0.24029, 0.30713, 0.32169, 0.016744, 0.037438, 0.002004, 0.15701, 0.005428, 0.000323, 0.002122, 3.8e-05, 0.13281, 0, 0.15856, 0 };
                sv[57] = new double[18] { 0.21907, 0.058074, 0.064764, 0.26253, 0.11472, 0.052631, 0.044436, 0.13352, 0.010103, 0.093875, 0.10917, 0.004005, 0.036369, 0.002157, 0.046875, 0, 0.008463, 0 };
                sv[58] = new double[18] { 0.35504, 0.34485, 0.29526, 0.33102, 0.39335, 0.32754, 0.18993, 0.2995, 0.067496, 0.27167, 0.0193, 0.000499, 0.013658, 0.00019, 0, 0, 0.93406, 0 };
                sv[59] = new double[18] { 0.38713, 0.24307, 0.11086, 0.42106, 0.26236, 0.10112, 0.27433, 0.20913, 0.069845, 0.66669, 0, 0, 0, 0, 0, 0, 0.34421, 0 };
                sv[60] = new double[18] { 0.19269, 0.2774, 0.30351, 0.20649, 0.29295, 0.32497, 0.010646, 0.028078, 0.001243, 0.13892, 0.010253, 0.000935, 0.004432, 0.000126, 0.16406, 0, 0.07982, 0 };
                sv[61] = new double[18] { 0.23336, 0.11706, 0.11157, 0.2549, 0.19072, 0.1057, 0.40286, 0.43989, 0.19229, 0.46116, 0.033172, 0.000777, 0.005658, 2.8e-05, 0.054688, 0, 0.11702, 0 };
                sv[62] = new double[18] { 0.17124, 0.20655, 0.16428, 0.2311, 0.25938, 0.18299, 0.05035, 0.13103, 0.010934, 0.12166, 0.058504, 0.005327, 0.013281, 0.00026, 0.15625, 0, 0.048666, 0 };
                sv[63] = new double[18] { 0.28176, 0.21098, 0.20994, 0.27031, 0.28344, 0.23897, 0.0378, 0.089792, 0.006562, 0.14324, 0.45115, 0.053702, 0.22271, 0.039695, 0.09375, 0, 0.14433, 0 };
                sv[64] = new double[18] { 0.35815, 0.29721, 0.24202, 0.39426, 0.37293, 0.25465, 0.040743, 0.074876, 0.006072, 0.19921, 0, 0, 0, 0, 0, 0, 0.017039, 0 };
                sv[65] = new double[18] { 0.56198, 0.56049, 0.30218, 0.56671, 0.57773, 0.29059, 0.22504, 0.23062, 0.062991, 0.47309, 0.19481, 0.02713, 0.17839, 0.017184, 0, 0, 0.23547, 0 };
                sv[66] = new double[18] { 0.3739, 0.26135, 0.13627, 0.37597, 0.28678, 0.12895, 0.30739, 0.24466, 0.088702, 0.64681, 0.30458, 0.13526, 0.19219, 0.033958, 0.53125, 0, 0.14641, 0 };
                sv[67] = new double[18] { 0.45527, 0.48785, 0.51574, 0.47717, 0.52331, 0.52953, 0.13329, 0.13103, 0.024744, 0.45958, 0.004222, 6.4e-05, 0.004526, 5.5e-05, 0, 0, 0.48747, 0 };
                sv[68] = new double[18] { 0.15092, 0.06264, 0.033227, 0.12158, 0.08757, 0.039535, 0.025629, 0.10295, 0.005584, 0.056801, 0.051267, 0.001754, 0.008298, 0.000237, 0.27344, 0, 0.15037, 0 };
                sv[69] = new double[18] { 0.55617, 0.33388, 0.14865, 0.55749, 0.34409, 0.14887, 0.31179, 0.24569, 0.090222, 0.65485, 0, 0, 0, 0, 0, 0, 0.068983, 0 };
                sv[70] = new double[18] { 0.13236, 0.16272, 0.16185, 0.13879, 0.21905, 0.1846, 0.10805, 0.26206, 0.036413, 0.13316, 0.11158, 0.003535, 0.024503, 0.000167, 0.13281, 0, 0.37068, 0 };
                sv[71] = new double[18] { 0.22306, 0.27585, 0.22137, 0.27688, 0.3048, 0.25244, 0.046527, 0.11231, 0.009109, 0.13861, 0.015078, 0.000557, 0.003348, 3.3e-05, 0, 0, 0.35752, 0 };
                sv[72] = new double[18] { 0.32863, 0.35472, 0.20099, 0.35148, 0.40541, 0.30572, 0.047008, 0.12167, 0.009779, 0.12351, 0.048854, 0.002075, 0.01108, 0.000285, 0.22656, 0, 0.1103, 0 };
                sv[73] = new double[18] { 0.23248, 0.045409, 0.063013, 0.24942, 0.11077, 0.062806, 0.028545, 0.11231, 0.00643, 0.056717, 0.14958, 0.006243, 0.038616, 0.001551, 0.32031, 0, 0.21231, 0 };
                sv[74] = new double[18] { 0.46761, 0.21847, 0.10652, 0.48747, 0.25143, 0.12264, 0.26277, 0.20839, 0.066879, 0.63625, 0.079011, 0.00798, 0.064266, 0.004854, 0.42969, 0, 0.37326, 0 };
                sv[75] = new double[18] { 0.54051, 0.45652, 0.20822, 0.54621, 0.50095, 0.11276, 0.10247, 0.18719, 0.02612, 0.21298, 0.088661, 0.005694, 0.078741, 0.005863, 0.41406, 0, 0.17101, 0 };
                sv[76] = new double[18] { 0.37128, 0.4382, 0.44947, 0.40601, 0.44981, 0.47467, 0.02861, 0.028078, 0.002503, 0.31275, 0.18396, 0.030151, 0.11992, 0.005278, 0.46875, 0, 0.31761, 0 };
                sv[77] = new double[18] { 0.4189, 0.22103, 0.13853, 0.47581, 0.19127, 0.13761, 0.31719, 0.24569, 0.091701, 0.66823, 0.098914, 0.02235, 0.039119, 0.002279, 0.38281, 0, 0.48633, 0 };
                weight = new double[nSV] { 
                    100, 100, 100, 100, 39.3799, 97.579, 100, 100, 94.3203, 53.612, 100, 49.2905, 63.141, 100, 8.5328, 100, 100, 42.5437, 100, 100, 100, 100, 49.0432, 76.8885, 100, 21.2943, 100, 98.5401, 100, 100, 0.32195, 100, 100, 2.4289, 100, 61.8281, 100, -100, -100, -100, -100, -100, -100, -81.3714, -20.8019, -100, -100, -100, -100, -23.3805, -53.4288, -100, -14.7264, -20.9565, -100, -100, -100, -100, -0.35801, -100, -100, -100, -16.0698, -17.6697, -27.8793, -46.2711, -56.6828, -39.3623, -100, -100, -83.0076, -100, -10.5659, -1.7836, -100, -100, -56.1291, -88.2994
                };


                
                feature_min = new double[18] { 
                    220996.7221, 68830.0606, 12746.3056, 895.4122, 6.1781, 4.407, 1345993.4864, 640, 919819954.0384, 1085.6126, 0, 0, 0, 0, -1, 0, 12961.689, 0
                };
                scalingFactors = new double[18] { 
                    6.5067e-07, 6.3128e-07, 6.7504e-07, 0.00016392, 0.00015872, 0.00018151, 1.5831e-08, 7.3121e-05, 1.0838e-12, 0.00015687, 0.00060314, 0.00039021, 1.5717e-05, 1.9774e-07, 0.0078125, 0, 5.4579e-07, 0 
                };


                //////////////////////////////////////////// paste ends
                // (f-feature_min)*scalingFactors , scalingFactors =1/(feature_min-feature_min)

                features = new double[18];
                features_normalized = new double[18];
            }
            public static void Normalize()
            {
                for (int i = 0; i < 18; i++)
                {
                    features_normalized[i] = (features[i] - feature_min[i]) * scalingFactors[i];
                }
            }
            public static double Decide(){
                double decision = 0;
                for (int i = 0; i < nSV; i++)
                {
                    decision += weight[i] * Kernel(features_normalized, sv[i]);
                }
                decision -= rho;
                return decision;
            }
            public static double Kernel(double[] features1, double[] features2)
            {
                double res = 0;

                double sum = 0;
                for (int i = 0; i < features1.Length; i++)
                {
                    double tmp = features1[i] - features2[i];
                    sum += tmp * tmp;
                }
                res = System.Math.Exp(-gamma * sum);//1 / (float)System.Math.Pow(System.Math.Pow(2 * System.Math.Sqrt(sum) * System.Math.Sqrt(System.Math.Pow(2, 1 / omega) - 1) / sigma, 2) + 1, omega);
                return res;
            }
        }

        ////////////////////////////////////////////////
        

        public HumanCarClassifier()
        { 
            Debug.Print("Initializing LCD ....");
            lcd = new EmoteLCD();
            lcd.Initialize();
            lcd.Clear();
            lcd.Write(LCD.CHAR_S, LCD.CHAR_T, LCD.CHAR_A, LCD.CHAR_R);
            

            Debug.Print("Initializing sample buffer arr ...");
            channelIBuffer = new BufferStorage(fftWindowSize, step);  //fftWindowSize, 
            channelQBuffer = new BufferStorage(fftWindowSize, step);  //fftWindowSize, 

            Debug.Print("Initializing ADC .....");
            adcCallbackPtr = AdcCallbackFn;
            AnalogInput.InitializeADC();
            if (!AnalogInput.ConfigureContinuousModeDualChannel(sampleBuffer1, sampleBuffer2, step, sampleTime, AdcCallbackFn))
            {
                throw new InvalidOperationException("ADC Initialization failed \n");
            }

            Debug.Print("Initializing DataStore .....");

            dStore = DataStore.Instance(StorageType.NOR);

            Debug.Print("Successfully Initialized DataStore.");

            dataRefArray = new DataReference[nDataRenference];
            dStore.ReadAllDataReferences(dataRefArray, 0);

            //timer = new Timer(BeaconTimerCallback,null,0,1000);

            dState.init();
            Phase.init();
            Polyfit.init();
            Spectrogram.init();
            DecisionFunction.init();


            //byte[] beaconMessage = new byte[1];

            //RoutingLayer.InitializeRoutingLayer(beaconMessage, 0, 1);
            /*
            byte[] beaconMessage = new byte[1];
            if (RoutingLayer.InitializeRoutingLayer(beaconMessage, 0, 1) == Samraksh_eMote_Net_Routing.DeviceStatus.Success){
                Debug.Print("Rounting initializing success");
                lcd.Write(Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_S, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_S, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_S, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_S);
            }
            else
                lcd.Write(Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_F, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_F, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_F, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_F);
            */
        }


        public void AdcCallbackFn(long threshold)
        {
            if (sampleBuffer1[0] <=4 && sampleBuffer1[1] <=4)
            {
                if (sampleBuffer2[0] <=4 && sampleBuffer2[1] <=4)
                {
                    AnalogInput.StopSampling();
                    Debug.Print("Stopping Experiment");
                    stopExperimentFlag = true;
                }
            }
            //Debug.Print("sampleBuffer1[0]=" + sampleBuffer1[0]);
            //Debug.Print("sampleBuffer1[0]=" + sampleBuffer2[0]);
            channelIBuffer.CopyFrom(sampleBuffer1);   // should comment out this two lines when use dummy data test
            channelQBuffer.CopyFrom(sampleBuffer2);
        }

        /* networking
        void BeaconTimerCallback(Object state)
        {
            //callbackTime.Write(true);
            //callbackTime.Write(false);
            //Debug.Print("Trying to send beacon# "+c);
            if (RoutingLayer.SendBeacon() == Samraksh_eMote_Net_Routing.DeviceStatus.Success)
            {
                Debug.Print("Send beacon success");
            }
            else
                Debug.Print("Send beacon failure");
            c++;
        }*/

        public void Run()
        {
            //lcd.Clear();
            //lcd.Write(Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_C, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_C, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_C, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_C);


            //while (dState.windowIndex<dState.nWindow)   //!stopExperimentFlag

            while (!stopExperimentFlag) //   //8 //dState.allWindowIndex< dState.nWindow
            {
                if (channelIBuffer.IsFull() && channelQBuffer.IsFull())       // comment out for dummy data test
                {
                    //Debug.Print("dState.allWindowIndex=" + dState.allWindowIndex);
                    if (dState.stopDisplayIndex == dState.allWindowIndex)
                        lcd.Write(LCD.CHAR_R, LCD.CHAR_U, LCD.CHAR_N, LCD.CHAR_N);

                    callbackTime.Write(true);
                    callbackTime.Write(false);
        
                    //Debug.Print("mem=" + Debug.GC(true));

                    //////////////////// update and rotate the sample array
                    Array.Copy(channelIBuffer.buffer, step, channelIBuffer.buffer, 0, fftWindowSize - step);
                    Array.Copy(channelQBuffer.buffer, step, channelQBuffer.buffer, 0, fftWindowSize - step);
                    
                    
                    ///// dummy radar samples for testing

                    // store data in RAM
                    /*
                    ushort[] dummyI = new ushort[] { 
                        2159, 2325, 2791, 2867, 3265, 3837, 4095, 4095, 4095, 4051, 3809, 3589, 3321, 3065, 2787, 2733, 2633, 2453, 2225, 1979, 1469, 1421, 1233, 1201, 1181, 1309, 1743, 1783, 1915, 2121, 2239, 2385, 2473, 2457, 2329, 2141, 2081, 1959, 2005, 2001, 2133, 2253, 2467, 2633, 2791, 2803, 2745, 2755, 2669, 2611, 2473, 2465, 2477, 2497, 2523, 2509, 2337, 2321, 2237, 2119, 1997, 1855, 1695, 1663, 1623, 1567, 1537, 1507, 1433, 1427, 1339, 1333, 1319, 1365, 1353, 1353, 1335, 1393, 1413, 1501, 1705, 1745, 1883, 2129, 2241, 2323, 2467, 2475, 2507, 2633, 2749, 2829, 2941, 2951, 2969, 2995, 3017, 3043, 3075, 3057, 2997, 2965, 2871, 2717, 2527, 2495, 2407, 2343, 2283, 2275, 2425, 2425, 2563, 2665, 2751, 2857, 3089, 3129, 3281, 3413, 3469, 3539, 3593, 3605, 3645, 3621, 3619, 3589, 3553, 3559, 3523, 3545, 3661, 3695, 3899, 3891, 3889, 3841, 3821, 3725, 3443, 3435, 3397, 3381, 3427, 3467, 3457, 3491, 3505, 3577, 3655, 3687, 3803, 3825, 3821, 3859, 3843, 3793, 3729, 3709, 3751, 3789, 3953, 3997, 3847, 3811, 3705, 3581, 3383, 3291, 3157, 3145, 3185, 3117, 3181, 3209, 3035, 3017, 2987, 2977, 2913, 2925, 2737, 2707, 2579, 2449, 2401, 2477, 2527, 2535, 2441, 2341, 2185, 2205, 2189, 2211, 2277, 2399, 2531, 2541, 2583, 2597, 2617, 2615, 2537, 2561, 2639, 2643, 2705, 2737, 2807, 2955, 3057, 3057, 3029, 3121, 3169, 3073, 2939, 2913, 2801, 2763, 2653, 2521, 2383, 2379, 2291, 2249, 2235, 2241, 2221, 2217, 2319, 2449, 2507, 2571, 2741, 2767, 2915, 2941, 2995, 3053, 3173, 3175, 3225, 3191, 3201, 3211, 3173, 3181, 3161, 3105, 3117, 3217, 3185, 3197, 3271, 3237, 3235, 3187, 2981, 2943, 2821, 2687, 2571, 2485, 2275, 2265, 2201, 2105, 1973, 1831, 1923, 1933, 1969, 1837, 1759, 1741, 1787, 1821, 1895, 2013, 2121, 2205, 2425, 2467, 2569, 2659, 2725, 2791, 2933, 2961, 3031, 3089, 3237, 3287, 3377, 3437, 3553, 3467, 3363, 3339, 3217, 3191, 3089, 3045, 2995, 3003, 2823, 2805, 2701, 2657, 2519, 2453, 2295, 2295, 2223, 2115, 2023, 1977, 1887, 1869, 1847, 1865, 1919, 1923, 1935, 1931, 1897, 1887, 1971, 2061, 2205, 2227, 2303, 2409, 2433, 2463, 2611, 2609, 2567, 2637, 2641, 2641, 2653, 2657, 2699, 2719, 2667, 2797, 2857, 2851, 2917, 2925, 2949, 2907, 2835, 2865, 2889, 2805, 2851, 2917, 2945, 2973, 3055, 3061, 3053, 3015, 3055, 3089, 3115, 3127, 3111, 3169, 3249, 3233, 3163, 3137, 3109, 3175, 3215, 3237, 3245, 3253, 3089, 3047, 2989, 3037, 3093, 3039, 2993, 2993, 2987, 3007, 3149, 3243, 3307, 3365, 3313, 3347, 3389, 3335, 3401, 3313, 3289, 3297, 3235, 3257, 3273, 3327, 3223, 3233, 3261, 3195, 3161, 3133, 3067, 3047, 3099, 3083, 3025, 3009, 2983, 2977, 3083, 3039, 2969, 2983, 3019, 3037, 3105, 3193, 3181, 3109, 3157, 3155, 3161, 3109, 2999, 3037, 2947, 2935, 2807, 2649, 2429, 2243, 1873, 1805, 1629, 1375, 1353, 1323, 1549, 1551, 1665, 1875, 2079, 2205, 2587, 2645, 2815, 2943, 2979, 2879, 2947, 2939, 3033, 2989, 2739, 2593, 2355, 2337, 2259, 2139, 2019, 1925, 1829, 1797, 1779, 1805, 1841, 1869, 1879, 1869, 1879, 1985, 2213, 2307, 2639, 2635, 2701, 2779, 2769, 2741, 2641, 2625, 2547, 2455, 2439, 2335, 2153, 2141, 2169, 2243, 2271, 2351, 2401, 2409, 2475, 2647, 2781, 2857, 3075, 3089, 3189, 3265, 3269, 3395, 3321, 3319, 3231, 3149, 3045, 2891, 2753, 2731, 2615, 2487, 2409, 2405, 2311, 2341, 2383, 2439, 2629, 2657, 2767, 2745, 2707, 2589, 2495, 2499, 2527, 2581, 2753, 2899, 2989, 3023, 2853, 2837, 2761, 2667, 2735, 2913, 3003, 3011, 2947, 2789, 2717, 2577, 2437, 2385, 2309, 2245, 2175, 2139, 2211, 2217, 2275, 2387, 2601, 2715, 2547, 2565, 2661, 2697, 2881, 3027, 3047, 3043, 2943, 2953, 2849, 2713, 2355, 2295, 2119, 1979, 1905, 1805, 1761, 1777, 1845, 2033, 2191, 2273, 2469, 2491, 2499, 2517, 2469, 2431, 2451, 2441, 2399, 2325, 2313, 2223, 2101, 2097, 2059, 2147, 2127, 2113, 2165, 2165, 2281, 2347, 2403, 2419, 2501, 2519, 2585, 2631, 2635, 2681, 2681, 2697, 2749, 2769, 2815, 2807, 2851, 2841, 2897, 2835, 2721, 2689, 2599, 2587
                    };
                    ushort[] dummyQ = new ushort[] { 
                        3007, 3115, 3463, 3519, 3641, 3599, 3425, 3121, 2539, 2489, 2387, 2279, 2213, 2139, 1957, 1905, 1787, 1561, 1349, 1239, 1279, 1311, 1467, 1711, 1889, 2037, 2143, 2165, 2183, 2235, 2329, 2499, 3043, 3111, 3363, 3505, 3633, 3627, 3515, 3491, 3435, 3371, 3241, 3235, 3319, 3331, 3331, 3283, 3149, 3043, 2971, 2991, 2975, 2875, 2825, 2685, 2547, 2511, 2413, 2351, 2271, 2175, 2121, 2111, 2123, 2135, 2211, 2273, 2391, 2399, 2377, 2297, 2217, 2155, 2131, 2129, 2147, 2183, 2187, 2259, 2471, 2513, 2673, 2811, 2927, 2985, 3017, 2987, 2961, 2943, 2885, 2881, 2909, 2909, 2897, 2863, 2851, 2869, 2861, 2863, 2915, 2941, 2969, 2965, 2921, 2925, 2951, 3009, 3001, 3005, 3059, 3065, 3123, 3185, 3189, 3199, 3265, 3267, 3343, 3399, 3379, 3393, 3129, 3107, 2995, 2943, 2831, 2799, 2817, 2819, 2837, 2871, 2869, 2877, 2711, 2673, 2619, 2467, 2365, 2217, 2099, 2089, 2055, 2027, 2067, 2125, 2191, 2197, 2201, 2253, 2267, 2283, 2383, 2365, 2407, 2427, 2503, 2597, 2799, 2843, 2951, 3073, 3153, 3217, 3303, 3319, 3421, 3497, 3579, 3783, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4031, 3745, 3719, 3653, 3617, 3499, 3495, 3107, 3057, 2821, 2631, 2519, 2403, 2087, 2049, 1945, 1843, 1719, 1623, 1549, 1511, 1407, 1319, 1243, 1215, 1261, 1247, 1337, 1413, 1559, 1693, 1869, 1903, 1985, 2081, 2171, 2231, 2215, 2195, 2213, 2203, 2169, 2171, 2055, 2027, 2027, 2011, 1963, 1891, 1855, 1825, 1761, 1733, 1685, 1709, 1771, 1811, 1859, 1839, 1877, 2027, 2227, 2259, 2369, 2515, 2551, 2539, 2627, 2649, 2639, 2727, 2823, 2725, 2699, 2733, 2823, 2851, 2973, 3029, 3239, 3251, 3273, 3325, 3263, 3233, 3259, 3255, 3131, 3079, 2943, 2867, 2769, 2775, 2841, 2785, 2699, 2563, 2373, 2361, 2311, 2339, 2373, 2357, 2389, 2385, 2419, 2531, 2643, 2609, 2425, 2423, 2495, 2489, 2479, 2527, 2551, 2531, 2575, 2601, 2503, 2497, 2345, 2331, 2301, 2175, 2007, 2017, 1953, 1949, 1909, 1859, 1827, 1803, 1787, 1789, 1917, 1945, 1999, 2019, 2239, 2281, 2321, 2435, 2493, 2543, 2641, 2687, 2823, 2825, 2905, 3023, 3121, 3133, 3073, 3075, 3071, 3037, 3025, 3025, 3059, 3101, 3087, 3139, 3127, 3101, 3025, 3025, 3015, 2993, 2973, 2951, 2903, 2881, 2943, 3083, 2949, 2951, 2931, 2895, 2877, 2799, 2697, 2683, 2629, 2609, 2577, 2567, 2531, 2511, 2553, 2577, 2551, 2593, 2723, 2723, 2763, 2849, 2917, 2859, 2793, 2801, 2745, 2643, 2611, 2621, 2659, 2663, 2675, 2681, 2495, 2511, 2599, 2639, 2821, 2797, 2781, 2803, 2983, 2981, 2977, 3017, 2991, 3079, 3007, 3015, 3025, 3063, 3193, 3257, 3319, 3347, 3297, 3299, 3289, 3285, 3133, 3129, 3015, 2999, 2925, 2863, 2761, 2749, 2849, 2785, 2731, 2715, 2681, 2677, 2651, 2637, 2719, 2787, 2935, 2937, 3057, 3115, 3127, 3149, 3309, 3357, 3481, 3523, 3593, 3591, 3377, 3331, 3227, 3011, 2875, 2577, 2283, 2239, 2105, 2051, 2081, 2171, 2349, 2355, 2417, 2509, 2731, 2903, 3005, 3029, 3211, 3479, 3651, 3691, 3717, 3719, 3699, 3585, 3537, 3475, 3233, 3185, 3077, 2899, 2779, 2569, 2323, 2323, 2227, 2265, 2365, 2431, 2567, 2575, 2681, 2607, 2635, 2703, 2735, 2735, 2765, 2717, 2641, 2627, 2467, 2455, 2353, 2275, 2169, 2137, 2211, 2197, 2117, 2101, 2029, 2048, 2227, 2259, 2453, 2557, 2685, 2829, 2997, 2989, 3073, 3181, 3233, 3275, 3227, 3215, 3229, 3169, 3025, 2979, 2713, 2683, 2551, 2455, 2489, 2505, 2695, 2713, 2779, 2763, 2747, 2577, 2409, 2401, 2411, 2535, 2627, 2829, 2985, 2995, 2955, 2913, 2837, 2795, 3133, 3161, 3233, 3351, 3395, 3325, 3297, 3281, 3221, 3193, 3087, 3043, 2647, 2627, 2495, 2469, 2437, 2451, 2603, 2655, 2765, 2889, 2891, 2839, 2965, 2963, 2999, 3193, 3215, 3379, 3369, 3303, 3143, 3051, 2921, 2723, 2479, 2467, 2383, 2373, 2447, 2391, 2567, 2583, 2699, 2739, 2697, 2807, 2795, 2809, 2775, 2835, 2853, 2743, 2659, 2627, 2545, 2457, 2439, 2327, 2211, 2203, 2215, 2147, 2175, 2231, 2207, 2217, 2259, 2327, 2315, 2255, 2347, 2361, 2419, 2479, 2551, 2529, 2675, 2685, 2803, 2835, 2831, 2715, 2561, 2543
                    };
                    Array.Copy(dummyI, step * dState.allWindowIndex, sampleBuffer1, 0, step);
                    Array.Copy(dummyQ, step * dState.allWindowIndex, sampleBuffer2, 0, step);
                     * /


                    /* read dummy data from NOR
                    dataRefArray[1].Read(sampleBuffer1, step * dState.allWindowIndex, sampleBuffer1.Length);
                    dataRefArray[2].Read(sampleBuffer2, step * dState.allWindowIndex, sampleBuffer2.Length);
                     */
                    
                    sampleBuffer1.CopyTo(channelIBuffer.buffer, fftWindowSize - step); 
                    sampleBuffer2.CopyTo(channelQBuffer.buffer, fftWindowSize - step);
                      
                    ///////////////////// rotate and unwrap the phase


                    dState.rotatePhase();
                    dState.clearMaxMinPhaseInStep();

                    dState.IsumInStep = 0;
                    dState.QsumInStep = 0;
                    for (int i = 0; i < step; i++)
                    {
                        dState.IsumInStep += channelIBuffer.buffer[256 - step + i];
                        dState.QsumInStep += channelQBuffer.buffer[256 - step + i];
                        int currPhase = Phase.unwrap((short)(channelIBuffer.buffer[256 - step + i] - meanI), (short)(channelQBuffer.buffer[256 - step + i] - meanQ));//>> 12; // /4096
                        if (currPhase > dState.maxPhaseInStep) dState.maxPhaseInStep = currPhase;
                        if (currPhase < dState.minPhaseInStep) dState.minPhaseInStep = currPhase;


                        dState.phase[128 - step + i] = currPhase;
                        //Debug.Print("currPhase=" + currPhase);
                    }
                    dState.updateMaxMinPhaseInStepArr();

                    
                    if (dState.allWindowIndex == int.MaxValue) dState.allWindowIndex=1200; //keep allWindowIndex always at int.maxvalue

                    dState.nStepForMeanIQ = (dState.allWindowIndex > 1200)? 1200: dState.allWindowIndex;   //1200=4*60*5  only consider the last 5 minutes data to estimate I and Q
                    meanI = (dState.IsumInStep + meanI * dState.nStepForMeanIQ * step) / (step + dState.nStepForMeanIQ * step);
                    meanQ = (dState.QsumInStep + meanQ * dState.nStepForMeanIQ * step) / (step + dState.nStepForMeanIQ * step);
                    //Debug.Print("newI=" + channelIBuffer.buffer[192]);
                    //Debug.Print("meanI =" + meanI);
                    //Debug.Print("meanQ =" + meanQ);*/

                    // wait for stable data stream
                    if (dState.allWindowIndex < 10)
                    {
                        channelIBuffer.bufferfull = false;
                        channelQBuffer.bufferfull = false;
                        dState.allWindowIndex++;
                        continue;
                    }


                    ////////////////////// above is done before step 10, wait for the radar data is stable


                    //Debug.Print("dState.phase[0]=" + dState.phase[0]);
                    //Debug.Print("dState.phase[64]=" + dState.phase[64]);
                    dState.phaseArrDiffofTwoEnds = System.Math.Abs(dState.phase[dState.phase.Length - 1] - dState.phase[0]);
                    dState.phaseChangeInOneStep = dState.phase[step] - dState.phase[0];
                    //Debug.Print("TwoEnds=" + dState.phaseArrDiffofTwoEnds);
                    
                    //Debug.Print("dState.validFlag=" + dState.validFlag);
                    //Debug.Print("dState.phaseChangeInOneStep=" + dState.phaseChangeInOneStep);
                    
                    if (dState.validFlag == 0)
                    {
                        if (System.Math.Abs(dState.phaseChangeInOneStep) > dState.thr_phaseChangeInOneStep)
                        {
                            dState.validFlag = 1;
                            //Debug.Print("Valid window started");
                        }
                        else
                        {
                            channelIBuffer.bufferfull = false;   // if continue, these three lines which are at bottom should be maintained
                            channelQBuffer.bufferfull = false;
                            dState.allWindowIndex++;
                            continue;  // no need to do the other work
                        }
                    }
                    else if (dState.validFlag == 1)
                    {
                        

                        dState.cumPhaseChange += dState.phaseChangeInOneStep;
                        dState.nStepInStateOne++;
                        if (dState.nStepInStateOne == 12)
                        {
                            Debug.Print("cumPhaseChange=" + dState.cumPhaseChange);
                            if (System.Math.Abs(dState.cumPhaseChange) > dState.thr_cumPhaseChange)
                            {
                                dState.validFlag = 2;
                            }
                            else
                            {
                                dState.validFlag = 0;
                                dState.validWindowIndex = 0;

                                // may need to add more. All intermediate variables to compute features need to be clear to 0 here.
                                dState.sum_phase_diff = 0;
                                Spectrogram.numHitBins_sum = 0;
                                Spectrogram.moment_sum = 0; 
                            }
                            dState.nStepInStateOne = 0;
                            dState.cumPhaseChange = 0;
                        }
                    }
                    else if (dState.validFlag == 2)
                    {
                        if (System.Math.Abs(dState.phaseChangeInOneStep) < dState.thr_phaseChangeInOneStep)
                        {
                            dState.stopWindowIndex++;
                        }
                        else
                        {
                            dState.stopWindowIndex = 0;
                        }
                        if (dState.stopWindowIndex == 12)
                        {  //12 step is 3 seconds. In 3 seconds there are no movement
                            dState.validFlag = 0;
                        }
                    }


                    /*     
                    if (dState.phaseArrDiffofTwoEnds > dState.thr_phaseArrDiffofTwoEnds)
                    {
                        dState.startWindowIndex++;
                        dState.stopWindowIndex = 0;
                    }
                    else
                    {
                        dState.startWindowIndex = 0;
                        dState.stopWindowIndex ++;
                    }*/

                    ////////////////////////////////////////////////////
                    dState.validFlag_last = dState.validFlag;

                    /*
                    if (dState.validFlag == 0)
                    {
                        if (dState.startWindowIndex == 3)
                        {
                            dState.validFlag = 1;
                            //Debug.Print("Valid window started");
                            //Debug.Print("mem = " + Debug.GC(true));
                        }
                        else
                        {
                            channelIBuffer.bufferfull = false;
                            channelQBuffer.bufferfull = false;
                            dState.allWindowIndex++;
                            //if (dState.allWindowIndex % 120 == 0) Debug.Print(dState.allWindowIndex/120+ "halfminute");
                            continue;   // do not do anything if validWindow does not started
                            
                        }
                    }
                    else if (dState.validFlag==2)
                    {
                        if (dState.stopWindowIndex == 5)
                        {
                            dState.validFlag = 0;
                            //Debug.Print("Valid window Stop!");
                        }
                    }
                     * */
                    ////////////////////////////////// classification
                    

                    if ((dState.validFlag == 0 && dState.validFlag_last==2) || dState.validWindowIndex == dState.nWindowMax)   //dState.allWindowIndex == dState.nWindow - 1
                    {
                        Debug.Print("triggered classification");
                        //callbackTime.Write(true);
                        //callbackTime.Write(false);


                        int index_90 = dState.validWindowIndex * 9 / 10;
                        int index_70 = dState.validWindowIndex * 7 / 10;
                        int index_50 = dState.validWindowIndex * 5 / 10;
                        int index_10 = dState.validWindowIndex / 10;


                        DecisionFunction.features[0] = dState.diffPhaseInWindowArr[index_90];
                        DecisionFunction.features[1] = dState.diffPhaseInWindowArr[index_70];
                        DecisionFunction.features[2] = dState.diffPhaseInWindowArr[index_50];

                        DecisionFunction.features[3] = dState.veloInWindowArr[index_90];
                        DecisionFunction.features[4] = dState.veloInWindowArr[index_70];
                        DecisionFunction.features[5] = dState.veloInWindowArr[index_50];

                        DecisionFunction.features[6] = dState.sum_phase_diff;
                        DecisionFunction.features[7] = dState.validWindowIndex*step;
                        DecisionFunction.features[8] = DecisionFunction.features[6] * DecisionFunction.features[7];
                        DecisionFunction.features[9] = DecisionFunction.features[6] / DecisionFunction.features[7];
                        DecisionFunction.features[10] = Spectrogram.numHitBins_sum;  //   / dState.nWindow   after displacement detection
                        DecisionFunction.features[11] = Spectrogram.var(Spectrogram.numHitBinsArr);
                        DecisionFunction.features[12] = Spectrogram.moment_sum;   //     / dState.nWindow    after displacement detection
                        DecisionFunction.features[13] = Spectrogram.var(Spectrogram.momentArr);


                        DecisionFunction.features[14] = Spectrogram.maxFreqThroughWindows;

                        //DecisionFunction.features[15] = Spectrogram.maxRunLenThroughWindows;
                        DecisionFunction.features[16] = dState.accInWindowArr[index_90];
                        //DecisionFunction.features[17] = dState.diffPhaseInWindowArr[index_90] - dState.diffPhaseInWindowArr[index_10];



                        
                        /////////////////////
                        /*
                        Debug.Print("phase[0]=" + dState.phase[0]);
                        Debug.Print("phase[10]=" + dState.phase[10]);
                        Debug.Print("phase[20]=" + dState.phase[20]);
                        Debug.Print("phase[30]=" + dState.phase[30]);
                        Debug.Print("phase[40]=" + dState.phase[40]);
                        Debug.Print("phase[60]=" + dState.phase[60]);
                        Debug.Print("phase[80]=" + dState.phase[80]);
                        Debug.Print("phase[100]=" + dState.phase[100]);
                        Debug.Print("phase[127]=" + dState.phase[127]);
                         * */
                        //////////////////



                        
                        for (int i = 0; i < 18; i++)
                        {
                            
                            //Debug.Print("f[" + i + "]=" + DecisionFunction.features[i]);
                        }



                        DecisionFunction.Normalize();
                        
                        for (int i = 0; i < 18; i++)
                        {
                            //Debug.Print("fn[" + i + "]=" + DecisionFunction.features_normalized[i]);
                        }
                        double decision = DecisionFunction.Decide();
                        //Debug.Print("decision = " + decision);

                        { 
                            if (decision > 0)
                            {
                                Debug.Print("Dog"); //class1;
                                lcd.Write(LCD.CHAR_D, LCD.CHAR_D, LCD.CHAR_D, LCD.CHAR_D);
                                dState.stopDisplayIndex = dState.allWindowIndex + 60;

                            }
                            else
                            {
                                Debug.Print("Human"); //class2
                                lcd.Write(LCD.CHAR_H, LCD.CHAR_H, LCD.CHAR_H, LCD.CHAR_H);
                                dState.stopDisplayIndex = dState.allWindowIndex + 60;
                            }

                            
                            /* networking
                            if (RoutingLayer.SendBeacon() == Samraksh_eMote_Net_Routing.DeviceStatus.Success)
                            {
                                //lcd.Write(Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_S, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_E, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_N, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_T);
                                //dState.stopDisplayIndex = dState.allWindowIndex + 12;
                                
                                Debug.Print("Sent classification result successfully");
                            }
                            else
                                Debug.Print("Sent classification result failure");
                             * */
                             
                        }
                        dState.allWindowIndex++;

                        // these are the intermediate variables need to reset in the process of valid data collection
                        dState.validWindowIndex = 0;
                        dState.sum_phase_diff = 0;
                        Spectrogram.numHitBins_sum = 0;
                        Spectrogram.moment_sum=0; 

                        channelIBuffer.bufferfull = false;
                        channelQBuffer.bufferfull = false;

                        //callbackTime.Write(true);
                        //callbackTime.Write(false);

                        continue;
                    }
                    /////////////////////// use phase to compute feature class 1, 
                    

                    dState.clearMaxMinPhaseInWindow();
                    for (int i = 0; i < dState.maxPhaseInStepArr.Length; i++)
                    {
                        if (dState.maxPhaseInStepArr[i] > dState.maxPhaseInWindow) dState.maxPhaseInWindow = dState.maxPhaseInStepArr[i];
                        if (dState.minPhaseInStepArr[i] < dState.minPhaseInWindow) dState.minPhaseInWindow = dState.minPhaseInStepArr[i];
                    }

                    dState.diffPhaseInWindow = dState.maxPhaseInWindow - dState.minPhaseInWindow;  //to Change
                    
                    //Debug.Print("dState.diffPhaseInWindow=" + dState.diffPhaseInWindow);
                  


                    ///////////////////// compute fft

                    //callbackTime.Write(true);
                    //callbackTime.Write(false);

                    for (int i = 0; i < fftWindowSize; i++)
                    {
                        //fftInput[i*2] = (short)(((channelIBuffer.buffer[i] - meanI)>>4) * hammingWindow[i]); 
                        //fftInput[i*2+1] = (short)(((channelQBuffer.buffer[i] - meanQ)>>4) * hammingWindow[i]); 
                        fftInput[i*2] = (short)(channelIBuffer.buffer[i] - meanI); 
                        fftInput[i*2+1] = (short)(channelQBuffer.buffer[i] - meanQ);
                    }

                    transforms.FFT(fftInput, fftOutput, (ushort)(2 * fftWindowSize));

                    ///////////////////////////// spectrgram related
                    Spectrogram.numHitBins = 0;
                    Spectrogram.moment = 0;
                    Spectrogram.MaxRunLenInWindow = 0;

                    bool[] hit = new bool[256];
                    for (int i = 0; i < 256; i++)
                    {
                        if (i >= 50 && i <= 205) continue;
                        int power = fftOutput[2 * i] * fftOutput[2 * i] + fftOutput[2 * i + 1] * fftOutput[2 * i + 1];
                        //if (power> Spectrogram.thr_sqr_Csharp[i])    // threshold sometimes changes 
                        if (power > Spectrogram.thr_sqr_Csharp)
                        {
                            Spectrogram.moment += Spectrogram.freq(i);
                            Spectrogram.numHitBins++;
                            if (Spectrogram.freq(i) > Spectrogram.maxFreqInWindow)
                            {
                                Spectrogram.maxFreqInWindow = Spectrogram.freq(i);
                            }

                        }

                        /*
                        int NumInWind = 0;
                        int RunLen = 0;
                        
                        if (i<=6){
                            if (power > Spectrogram.thr_sqr_Csharp[i])
                            {
                                NumInWind++;
                                  
                            }
                        }
                        if (i==6){
                            if (NumInWind>=4) 
                                RunLen = 7;
                            else
                                RunLen = 0;
                        }
                        Spectrogram.MaxRunLenInWindow = RunLen;
                        if (i > 7)
                        {
                            int j = i - 7;
                            int power_before7 = fftOutput[2 * j] * fftOutput[2 * j] + fftOutput[2 * j + 1] * fftOutput[2 * j + 1];
                            if (power_before7 > Spectrogram.thr_sqr_Csharp[i])
                            {
                                NumInWind--;
                            }
                            if (power > Spectrogram.thr_sqr_Csharp[i])
                            {
                                NumInWind++;
                            }
                            if (NumInWind >= 4)
                            {
                                if (RunLen == 0) RunLen = 4;
                                else
                                    RunLen++;
                            }
                            else RunLen = 0;
                            if (Spectrogram.MaxRunLenInWindow < RunLen)
                            {
                                Spectrogram.MaxRunLenInWindow = RunLen;
                            }
                         
                        }*/

                    
                    }

                    //Debug.Print("numHitBins="+Spectrogram.numHitBins);

                    if (Spectrogram.MaxRunLenInWindow > Spectrogram.maxRunLenThroughWindows) Spectrogram.maxRunLenThroughWindows = Spectrogram.MaxRunLenInWindow;

                    
                    if (Spectrogram.maxFreqInWindow > Spectrogram.maxFreqThroughWindows) Spectrogram.maxFreqThroughWindows = Spectrogram.maxFreqInWindow;


                    Spectrogram.numHitBinsArr[dState.validWindowIndex] = Spectrogram.numHitBins;  //dState.allWindowIndex-3
                    //Debug.Print("Spectrogram.numHitBins=" + Spectrogram.numHitBins);
                    Spectrogram.numHitBins_sum += Spectrogram.numHitBins;

                    Spectrogram.momentArr[dState.validWindowIndex] = Spectrogram.moment;  //dState.allWindowIndex-3
                    Spectrogram.moment_sum += Spectrogram.moment;
                

                    ////////////////// polyfit1



                    dState.veloInWindow = Polyfit.polyfit1(dState.phase);

                    ///////////////////// polyfit3


                    Polyfit.addPoint(dState.phase);
                    Polyfit.getBestFit();
                    double A0 = Polyfit.polyparams[0];
                    double A1 = Polyfit.polyparams[1];
                    double A2 = Polyfit.polyparams[2];
                    double A3 = Polyfit.polyparams[3];
                    //Debug.Print(" A0 = " + A0 + " A1 = " + A1 + " A2= " + A2 + " A3= " + A3);

                    int Sign0 = 2*A2 + 3*A3 >= 0 ? 1 : -1;
                    int Sign1 = 2*A2 - 3*A3 >= 0 ? 1 : -1;
    
                    if (Sign0 == Sign1)
                        dState.accInWindow = System.Math.Abs(2*A2);
                    else
                        dState.accInWindow = System.Math.Abs(2 * A2 / 3 + 3 * A3 / 2);

                    ///////////////////////////////////////// insertion sort

                    int diffPhaseFlag = 0;
                    int veloFlag = 0;
                    int accFlag = 0;
                    for (int i = 0; i < dState.validWindowIndex; i++)  // because window 3 is window 0, the first 3 is thrown
                    {
                        if (diffPhaseFlag == 0 && dState.diffPhaseInWindowArr[i] > dState.diffPhaseInWindow)
                        {
                            Array.Copy(dState.diffPhaseInWindowArr, i, dState.diffPhaseInWindowArr, i + 1, dState.validWindowIndex - i);
                            dState.diffPhaseInWindowArr[i] = dState.diffPhaseInWindow;
                            diffPhaseFlag = 1;
                        }

                        if (veloFlag == 0 && dState.veloInWindowArr[i] > dState.veloInWindow)
                        {
                            Array.Copy(dState.veloInWindowArr, i, dState.veloInWindowArr, i + 1, dState.validWindowIndex - i);
                            dState.veloInWindowArr[i] = dState.veloInWindow;
                            veloFlag = 1;
                        }

                        if (accFlag == 0 && dState.accInWindowArr[i] > dState.accInWindow)
                        {
                            Array.Copy(dState.accInWindowArr, i, dState.accInWindowArr, i + 1, dState.validWindowIndex - i);
                            dState.accInWindowArr[i] = dState.accInWindow;
                            accFlag = 1;
                        }

                    }
                    if (diffPhaseFlag == 0) dState.diffPhaseInWindowArr[dState.validWindowIndex] = dState.diffPhaseInWindow;
                    if (veloFlag == 0) dState.veloInWindowArr[dState.validWindowIndex] = dState.veloInWindow;
                    if (accFlag == 0) dState.accInWindowArr[dState.validWindowIndex] = dState.accInWindow;
                    
                    

                    
                    channelIBuffer.bufferfull = false;
                    channelQBuffer.bufferfull = false;
                    dState.validWindowIndex++;
                    dState.allWindowIndex++;
                }
                //Debug.Print("Spectrogram.maxFreqThroughWindows=" + Spectrogram.maxFreqThroughWindows);

                Thread.Sleep(5);
            }

            

            // finished
            //lcd.Write(Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_D, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_D, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_D, Samraksh.SPOT.Hardware.EmoteDotNow.LCD.CHAR_D);
            Debug.Print("Experiment Complete\n");
        }

        public static void Main()
        {

            Debug.EnableGCMessages(false);
            Debug.Print("***Sleeping...***");

            Thread.Sleep(1000);

            //Debug.Print("mem_initial=" + Debug.GC(true));
            Debug.Print("***Starting Program...***");
            
            /* networking
            byte[] beaconMessage = new byte[1];
            RoutingLayer.InitializeRoutingLayer(beaconMessage, 11, 0);
            Debug.Print("Routing layer initialized successfully.");
             * */

            HumanCarClassifier hcl = new HumanCarClassifier();
            
            
            //int c = 0;
            
            
            /*while (true)
            {
                //RoutingLayer.SendBeacon();
                //Debug.Print("Sent beacon#");
                //c++;

                Thread.Sleep(1000);
            }*/
            
            

            hcl.Run();

        }

    }
}
