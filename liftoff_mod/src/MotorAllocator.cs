using System;
using System.Linq;

namespace GoProGeometry
{
    // Forces are FL, FR, RL, RR; moments are about Unity body X (pitch), Z (roll).
    // The fourth equation balances opposite propeller spin pairs. Liftoff still supplies yaw.
    public sealed class MotorAllocator
    {
        readonly double[,] inverse = new double[4,4];
        public readonly double[] HoverShares = new double[4];
        public MotorAllocator(double[][] points, double[] cg)
        {
            var a=new double[4,8];
            for(int i=0;i<4;i++) {
                a[0,i]=1; a[1,i]=-(points[i][2]-cg[2]); a[2,i]=points[i][0]-cg[0];
                a[3,i]=(i==0 || i==3)?1:-1; a[i,i+4]=1;
            }
            for(int col=0;col<4;col++) {
                int pivot=col;
                for(int row=col+1;row<4;row++) if(Math.Abs(a[row,col])>Math.Abs(a[pivot,col])) pivot=row;
                if(Math.Abs(a[pivot,col])<1e-9) throw new ArgumentException("Motor geometry cannot independently control pitch and roll.");
                for(int j=0;j<8;j++) { double t=a[col,j]; a[col,j]=a[pivot,j]; a[pivot,j]=t; }
                double divisor=a[col,col]; for(int j=0;j<8;j++) a[col,j]/=divisor;
                for(int row=0;row<4;row++) if(row!=col) {
                    double f=a[row,col]; for(int j=0;j<8;j++) a[row,j]-=f*a[col,j];
                }
            }
            for(int i=0;i<4;i++) {
                for(int j=0;j<4;j++) inverse[i,j]=a[i,j+4];
                HoverShares[i]=inverse[i,0];
                if(HoverShares[i]<.001 || double.IsNaN(HoverShares[i]))
                    throw new ArgumentException("CG must allow positive thrust at all four motors with balanced spin pairs.");
            }
        }
        public double Allocate(double collective, double pitch, double roll, double minimum, double maximum, double[] forces)
        {
            if(double.IsNaN(collective+pitch+roll+minimum+maximum) || double.IsInfinity(collective+pitch+roll+minimum+maximum) || minimum>maximum)
                throw new ArgumentException("Invalid motor force request.");
            var differential=new double[4];
            for(int i=0;i<4;i++) differential[i]=inverse[i,1]*pitch+inverse[i,2]*roll;
            bool Interval(double scale,out double low,out double high) {
                low=double.NegativeInfinity; high=double.PositiveInfinity;
                for(int i=0;i<4;i++) {
                    low=Math.Max(low,(minimum-scale*differential[i])/HoverShares[i]);
                    high=Math.Min(high,(maximum-scale*differential[i])/HoverShares[i]);
                }
                return low<=high;
            }
            if(!Interval(0,out _,out _)) throw new ArgumentException("Idle motor thrust is too high to balance this CG within the motor limits.");
            double authority=1;
            if(!Interval(1,out _,out _)) {
                double lo=0,hi=1;
                for(int i=0;i<40;i++) { double mid=(lo+hi)*.5; if(Interval(mid,out _,out _))lo=mid;else hi=mid; }
                authority=lo;
            }
            Interval(authority,out double lower,out double upper);
            double total=Math.Max(lower,Math.Min(upper,collective));
            for(int i=0;i<4;i++) forces[i]=Math.Max(minimum,Math.Min(maximum,HoverShares[i]*total+authority*differential[i]));
            return authority;
        }
        public static MotorAllocator FromProfile(GeometryProfile p) => new MotorAllocator(
            Enumerable.Range(0,4).Select(i=>GeometryMath.UnityPoint(p.motorsM,3*i)).ToArray(),GeometryMath.UnityPoint(p.centerOfMassM));
    }
}
