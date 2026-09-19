using System;
using System.Linq;

namespace GoProGeometry
{
    // Source coordinates: left, backward, up. Motor order: FL, FR, RL, RR.
    [Serializable]
    public sealed class GeometryProfile
    {
        public int schema = 1;
        public string name = "GoPro Drone";
        public string axes = "left_backward_up";
        public float massKg;
        public float[] centerOfMassM;
        public float[] motorsM;
        public float[] inertiaKgM2;
        public string notes;

        public void Validate()
        {
            if (schema != 1 || axes != "left_backward_up") throw new ArgumentException("Unsupported profile schema or axes.");
            if (string.IsNullOrWhiteSpace(name)) throw new ArgumentException("Profile needs a name.");
            if (!Finite(massKg) || massKg < 0.05f || massKg > 10f) throw new ArgumentException("Mass must be 0.05–10 kg.");
            Check(centerOfMassM, 3, "Center of gravity");
            Check(motorsM, 12, "Motor positions");
            Check(inertiaKgM2, 9, "Inertia tensor");
            if (motorsM.Any(x => Math.Abs(x) > 2f) || centerOfMassM.Any(x => Math.Abs(x) > 2f))
                throw new ArgumentException("Positions are in meters, with a 2 m limit.");
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
                if (Math.Abs(inertiaKgM2[3*i+j] - inertiaKgM2[3*j+i]) > 1e-8)
                    throw new ArgumentException("Inertia tensor must be symmetric.");
            var e = GeometryMath.PrincipalAxes(GeometryMath.UnityTensor(inertiaKgM2));
            if (e.values.Any(x => x < 1e-7 || x > 10)) throw new ArgumentException("Inertia must be positive definite.");
            for (int i = 0; i < 3; i++)
                if (e.values[i] > e.values[(i+1)%3] + e.values[(i+2)%3] + 1e-8)
                    throw new ArgumentException("Inertia violates the physical triangle inequality.");
            // Explicit labels prevent accidental mirroring/reordering in imported profiles.
            if (!(motorsM[0] > 0 && motorsM[3] < 0 && motorsM[6] > 0 && motorsM[9] < 0 &&
                  motorsM[1] < motorsM[7] && motorsM[4] < motorsM[10]))
                throw new ArgumentException("Motor order must be FL, FR, RL, RR in left/back/up coordinates.");
            for (int i = 0; i < 4; i++) for (int j = i+1; j < 4; j++)
            {
                double d = 0;
                for (int k = 0; k < 3; k++) d += Math.Pow(motorsM[3*i+k] - motorsM[3*j+k], 2);
                if (d < 0.0004) throw new ArgumentException("Motor centers must be at least 20 mm apart.");
            }
        }
        static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        static void Check(float[] a, int length, string label)
        {
            if (a == null || a.Length != length || a.Any(x => !Finite(x)))
                throw new ArgumentException(label + " has missing or invalid values.");
        }
    }

    public static class GeometryMath
    {
        // Unity body coordinates: right, up, forward. Tensor transform is A I A^T.
        public static double[] UnityPoint(float[] a, int offset = 0) => new double[] { -a[offset], a[offset+2], -a[offset+1] };
        public static double[,] UnityTensor(float[] i)
        {
            int[] order = { 0, 2, 1 }; int[] sign = { -1, 1, -1 };
            var r = new double[3,3];
            for (int a=0; a<3; a++) for (int b=0; b<3; b++) r[a,b] = sign[a]*sign[b]*i[3*order[a]+order[b]];
            return r;
        }
        // Jacobi diagonalization: columns of rotation are principal axes.
        public static (double[] values, double[,] rotation) PrincipalAxes(double[,] tensor)
        {
            var a = (double[,])tensor.Clone();
            var v = new double[,] {{1,0,0},{0,1,0},{0,0,1}};
            for (int step=0; step<40; step++)
            {
                int p=0, q=1;
                for (int x=0; x<3; x++) for (int y=x+1; y<3; y++)
                    if (Math.Abs(a[x,y]) > Math.Abs(a[p,q])) { p=x; q=y; }
                if (Math.Abs(a[p,q]) < 1e-14) break;
                double angle = 0.5*Math.Atan2(2*a[p,q], a[q,q]-a[p,p]);
                double c=Math.Cos(angle), s=Math.Sin(angle);
                double app=a[p,p], aqq=a[q,q], apq=a[p,q];
                a[p,p]=c*c*app-2*s*c*apq+s*s*aqq;
                a[q,q]=s*s*app+2*s*c*apq+c*c*aqq;
                a[p,q]=a[q,p]=0;
                for (int k=0; k<3; k++)
                {
                    if (k != p && k != q) {
                        double kp=a[k,p], kq=a[k,q];
                        a[k,p]=a[p,k]=c*kp-s*kq; a[k,q]=a[q,k]=s*kp+c*kq;
                    }
                    double vp=v[k,p], vq=v[k,q];
                    v[k,p]=c*vp-s*vq; v[k,q]=s*vp+c*vq;
                }
            }
            return (new[] {a[0,0], a[1,1], a[2,2]}, v);
        }
        public static double[] Moment(double[] point, double[] cg, double thrust)
            => new[] { -(point[2]-cg[2])*thrust, 0.0, (point[0]-cg[0])*thrust };
    }
}
