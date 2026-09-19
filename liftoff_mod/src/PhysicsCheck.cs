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
        internal static void Run(ManualLogSource log)
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
            }
            finally
            {
                if(root) UnityEngine.Object.DestroyImmediate(root);
                SceneManager.UnloadSceneAsync(scene);
            }
        }
    }
}
