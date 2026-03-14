// Name: Connor Churcott
// StudentID: 301553876

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using JetBrains.Annotations;
using Unity.VisualScripting;
using UnityEditor.Animations;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SocialPlatforms;

public static class ForwardKinematics
{

    public static void UpdateJointPositions(Actor actor, float[] frameData)
    {
        /*** Please write your Forward Kinematics code here ***/
        
        /*** code to be completed by students begins ***/

        int frameDataIndex = 0; 

        Quaternion EulerToQuat(Vector3 eulerLocalRot, Joint node)
        {
            Quaternion localQuat = new Quaternion(); 
            // Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.right); 
            // Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.up); 
            // Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.forward); 


            if(node.RotateOrder == Joint.RotationOrder.XYZ)
            {
                Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.right); 
                Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.up); 
                Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.forward); 
                
                localQuat = quatx * quaty * quatz; 
                //localQuat = quatz * quaty * quatx;
            }
            else if(node.RotateOrder == Joint.RotationOrder.XZY)
            {
                Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.right); 
                Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.up); 
                Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.forward); 

                localQuat = quatx * quatz * quaty; 
                //localQuat = quaty * quatz * quatx;
            }
            else if(node.RotateOrder == Joint.RotationOrder.YXZ)
            {
                Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.right); 
                Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.up); 
                Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.forward); 
                localQuat = quaty * quatx * quatz;
                //localQuat = quatz * quatx * quaty; 
            }
            else if(node.RotateOrder == Joint.RotationOrder.YZX)
            {
                Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.right); 
                Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.up); 
                Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.forward);
                localQuat = quaty * quatz * quatx; 
                //localQuat = quatx * quatz * quaty; 
            }
            else if(node.RotateOrder == Joint.RotationOrder.ZXY)
            {
                Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.right); 
                Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.up); 
                Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.forward);
                localQuat = quatz * quatx * quaty; 
                //localQuat = quaty * quatx * quatz;
            }
            else if(node.RotateOrder == Joint.RotationOrder.ZYX)
            {
                Quaternion quatx = Quaternion.AngleAxis(eulerLocalRot[2], Vector3.right); 
                Quaternion quaty = Quaternion.AngleAxis(eulerLocalRot[1], Vector3.up); 
                Quaternion quatz = Quaternion.AngleAxis(eulerLocalRot[0], Vector3.forward);
                localQuat = quatz * quaty * quatx; 
                //localQuat = quatx * quaty * quatz; 
            }
            else
            {
                localQuat = Quaternion.identity; 
            }
            return localQuat;  
        }

        List<float> ExtractFrameData(Joint node)
        {
            List<float> values = new List<float>();     
            int valuesTaken = 0; 
        
            if(node.ParentIdx == -1)
            {
                valuesTaken = 6; 
            }
            else if(node.RotateOrder != Joint.RotationOrder.NONE)
            {
                valuesTaken = 3; 
            }
            else
            {
                valuesTaken = 0; 
            }

            for(int i = 0; i < valuesTaken; i++)
            {
                values.Add(frameData[frameDataIndex]); 
                frameDataIndex++; 
            }

            return values; 
        }

        for(int i = 0; i < actor.Joints.Count; i++)
        {
            Joint curNode = actor.Joints[i]; 
            List<float> curFrameValues = ExtractFrameData(curNode); 

            if(curNode.ParentIdx == -1)
            {
                Vector3 localEuler = new Vector3(curFrameValues[3], curFrameValues[4], curFrameValues[5]); 
                Quaternion localQuat = EulerToQuat(localEuler, curNode);                
                Vector3 localPos = new Vector3(curFrameValues[0], curFrameValues[1], curFrameValues[2]); 

                curNode.LocalQuaternion = localQuat; 
                curNode.GlobalQuaternion = localQuat; 
                curNode.GlobalPosition = localPos; 
            }
            else if(curNode.RotateOrder != Joint.RotationOrder.NONE)
            {
                Vector3 localEuler = new Vector3(curFrameValues[0], curFrameValues[1], curFrameValues[2]); 
                Quaternion localQuat = EulerToQuat(localEuler, curNode); 

                curNode.LocalQuaternion = localQuat; 
                curNode.GlobalQuaternion = curNode.GetParent().GlobalQuaternion * localQuat; 
                curNode.GlobalPosition = curNode.GetParent().GlobalPosition + curNode.GetParent().GlobalQuaternion * curNode.LocalPosition; 
            }
            else
            {
                Quaternion localQuat = Quaternion.identity; 
                curNode.LocalQuaternion = localQuat; 
                curNode.GlobalQuaternion = curNode.GetParent().GlobalQuaternion * localQuat; 
                curNode.GlobalPosition = curNode.GetParent().GlobalPosition + curNode.GetParent().GlobalQuaternion * curNode.LocalPosition; 
            }
        }


        /*** code to be completed by students ends ***/
    }
}
