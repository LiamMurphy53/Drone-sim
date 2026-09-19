using System;
using System.Linq;
using System.Reflection;
using BepInEx.Logging;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace GoProGeometry
{
    internal static class PhysicsCheck
    {
        // Exercise Liftoff's real Propeller method in an isolated physics scene.
        // No game drone, global time step or global physics mode is changed.
        internal static void Run(ManualLogSource log,GeometryProfile profile)
        {
            var scene=SceneManager.CreateScene("GeometryLabForceCheck",new CreateSceneParameters(LocalPhysicsMode.Physics3D));
            GameObject root=null;
            try
            {
                root=new GameObject("GeometryLabTestBody"); SceneManager.MoveGameObjectToScene(root,scene);
                var rb=root.AddComponent<Rigidbody>(); rb.useGravity=false; rb.drag=0; rb.angularDrag=0; rb.mass=1;
                rb.centerOfMass=Vector3.zero; rb.inertiaTensor=new Vector3(.002f,.003f,.004f); rb.inertiaTensorRotation=Quaternion.identity; rb.maxAngularVelocity=1000;
                var child=new GameObject("GeometryLabTestMotor"); child.transform.SetParent(root.transform,false);
                var prop=child.AddComponent<Propeller>(); prop.enabled=false;
                typeof(Propeller).GetField("droneRigidbody",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(prop,rb);
                const float dt=.002f;
                Vector3 Step(Vector3 point)
                {
                    rb.position=Vector3.zero; rb.rotation=Quaternion.identity; rb.velocity=Vector3.zero; rb.angularVelocity=Vector3.zero;
                    child.transform.localPosition=point;
                    prop.ApplyControllerForceAtPropeller(1f);
                    scene.GetPhysicsScene().Simulate(dt);
                    return rb.angularVelocity;
                }
                var first=Step(new Vector3(.1f,0,0));
                var second=Step(new Vector3(.2f,0,0));
                var front=Step(new Vector3(0,0,.1f));
                if(Math.Abs(first.z-.05f)>.0001f || Math.Abs(second.z-.10f)>.0001f || Math.Abs(front.x+.10f)>.0001f ||
                   Math.Abs(rb.velocity.y-.002f)>.00001f)
                    throw new InvalidOperationException("Propeller force-path check failed: "+first+", "+second+", "+front);
                log.LogInfo("PASS real Liftoff Propeller force test: 100 mm arm = "+first.z.ToString("F6")+" rad/s; 200 mm arm = "+second.z.ToString("F6")+" rad/s; forward arm pitch = "+front.x.ToString("F6")+" rad/s.");
                // Use the game's native PID and real force method with the full GoPro tensor.
                var eig=GeometryMath.PrincipalAxes(GeometryMath.UnityTensor(profile.inertiaKgM2));
                Vector3 V(double[] a)=>new Vector3((float)a[0],(float)a[1],(float)a[2]);
                Vector3 C(int j)=>new Vector3((float)eig.rotation[0,j],(float)eig.rotation[1,j],(float)eig.rotation[2,j]);
                rb.mass=profile.massKg; rb.centerOfMass=V(GeometryMath.UnityPoint(profile.centerOfMassM));
                rb.inertiaTensor=V(eig.values); rb.inertiaTensorRotation=Quaternion.LookRotation(C(2),C(1));
                var props=new Propeller[4]; props[0]=prop;
                for(int i=0;i<4;i++) {
                    if(i>0) {
                        var motor=new GameObject("GeometryLabMotor"+i); motor.transform.SetParent(root.transform,false);
                        props[i]=motor.AddComponent<Propeller>(); props[i].enabled=false;
                        typeof(Propeller).GetField("droneRigidbody",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(props[i],rb);
                    }
                    props[i].transform.localPosition=V(GeometryMath.UnityPoint(profile.motorsM,3*i));
                }
                var tensor=GeometryMath.UnityTensor(profile.inertiaKgM2);
                var allocator=MotorAllocator.FromProfile(profile); var force=new double[4];
                float step=Time.fixedDeltaTime;
                float Trial(Vector3 initial,bool bias) {
                    rb.position=Vector3.zero; rb.rotation=Quaternion.identity; rb.velocity=Vector3.zero; rb.angularVelocity=initial;
                    var pitch=ControllerAdapter.TestPid(TaitBryanAngle.PITCH,(float)tensor[0,0]);
                    var roll=ControllerAdapter.TestPid(TaitBryanAngle.ROLL,(float)tensor[2,2]);
                    var yaw=ControllerAdapter.TestPid(TaitBryanAngle.YAW,(float)tensor[1,1]);
                    float peak=0;
                    for(int n=0;n<(int)(4/step);n++) {
                        var omega=rb.transform.InverseTransformDirection(rb.angularVelocity);
                        float Output(object pid,float rate)=>(float)ControllerAdapter.Calculate.Invoke(pid,new object[]{0f,rate});
                        double thrust=n<(int)(.3/step)?0:n<(int)(.6/step)?24:profile.massKg*9.80665;
                        allocator.Allocate(thrust,-Output(pitch,omega.x),-Output(roll,omega.z),.24,6,force);
                        for(int i=0;i<4;i++) props[i].ApplyControllerForceAtPropeller((float)force[i]);
                        rb.AddRelativeTorque(new Vector3(bias?.02f:0,Output(yaw,omega.y),0));
                        scene.GetPhysicsScene().Simulate(step);
                        peak=Math.Max(peak,rb.angularVelocity.magnitude);
                        if(float.IsNaN(peak) || peak>20) throw new InvalidOperationException("Controller check diverged.");
                    }
                    if(rb.angularVelocity.magnitude>.01f) throw new InvalidOperationException("Controller did not settle: "+rb.angularVelocity);
                    return peak;
                }
                float neutral=Trial(Vector3.zero,false);
                if(neutral>.005f) throw new InvalidOperationException("Throttle changes caused unwanted rotation: "+neutral);
                float recovery=Trial(new Vector3(2,0,-1),false);
                Trial(Vector3.zero,true);
                log.LogInfo("PASS GoPro controller physics: neutral throttle steps peak="+neutral.ToString("F6")+
                    " rad/s; 2 rad/s pitch + roll disturbance settled; constant 0.02 Nm pitch bias rejected. Full tensor and native PID/propeller force path used.");

            }
            finally
            {
                if(root) UnityEngine.Object.DestroyImmediate(root);
                SceneManager.UnloadSceneAsync(scene);
            }
        }
    }
}
