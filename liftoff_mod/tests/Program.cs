using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using GoProGeometry;

var path=args.Length>0?args[0]:"liftoff_mod/profiles/gopro-drone.json";
GeometryProfile Load()=>JsonSerializer.Deserialize<GeometryProfile>(File.ReadAllText(path),new JsonSerializerOptions {IncludeFields=true});
void Check(bool value,string message) { if(!value) throw new Exception(message); Console.WriteLine("PASS "+message); }
void Reject(Action<GeometryProfile> change,string label) { var p=Load(); change(p); bool failed=false; try{p.Validate();}catch(ArgumentException){failed=true;} Check(failed,label); }
var profile=Load();profile.Validate();
var tensor=GeometryMath.UnityTensor(profile.inertiaKgM2);
var eig=GeometryMath.PrincipalAxes(tensor);
double error=0;
for(int i=0;i<3;i++)for(int j=0;j<3;j++) {
    double reconstructed=Enumerable.Range(0,3).Sum(k=>eig.rotation[i,k]*eig.values[k]*eig.rotation[j,k]);
    error=Math.Max(error,Math.Abs(tensor[i,j]-reconstructed));
}
Check(error<1e-12,"full inertia tensor reconstructed from principal axes");
var v=eig.rotation;
double determinant=v[0,0]*(v[1,1]*v[2,2]-v[1,2]*v[2,1])-v[0,1]*(v[1,0]*v[2,2]-v[1,2]*v[2,0])+v[0,2]*(v[1,0]*v[2,1]-v[1,1]*v[2,0]);
Check(Math.Abs(determinant-1)<1e-12,"principal axes form a proper rotation");
var fl=GeometryMath.UnityPoint(profile.motorsM);
Check(fl[0]<0 && fl[2]>0,"front-left stays front-left in Unity");
var cg=GeometryMath.UnityPoint(profile.centerOfMassM);
double[] moment=new double[3];
for(int i=0;i<4;i++){var m=GeometryMath.Moment(GeometryMath.UnityPoint(profile.motorsM,i*3),cg,1);for(int k=0;k<3;k++)moment[k]+=m[k];}
Check(Math.Abs(moment[0]-.073504)<1e-7,"deadcat equal-thrust pitch moment preserves CG offset");
Check(Math.Abs(moment[2]-.000084)<1e-7,"small lateral CG offset is preserved");
var a=GeometryMath.Moment(new double[]{.1,0,0},new double[3],1);
var b=GeometryMath.Moment(new double[]{.2,0,0},new double[3],1);
Check(Math.Abs(b[2]-2*a[2])<1e-12,"doubling motor lever arm doubles moment");
Reject(p=>p.massKg=float.NaN,"reject non-finite mass");
Reject(p=>p.motorsM[0]=float.PositiveInfinity,"reject non-finite motor position");
Reject(p=>p.inertiaKgM2[0]=-1,"reject negative inertia");
Reject(p=>p.inertiaKgM2[1]=1,"reject nonsymmetric inertia");
Reject(p=>p.inertiaKgM2=new float[]{.01f,0,0,0,.001f,0,0,0,.001f},"reject impossible principal moments");
Reject(p=>p.axes="right_forward_up","reject ambiguous coordinate convention");
Reject(p=>p.motorsM[0]=-p.motorsM[0],"reject incorrect motor order");
