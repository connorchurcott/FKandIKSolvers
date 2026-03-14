// Name: Connor Churcott
// StudentID: 301553876

using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEditor.MPE;
using UnityEngine;

public static class InverseKinematics
{
    public static void ApplyIK(Actor actor, Joint endEffector, Vector3 targetPosition)
    {
        /*** Please write your Inverse Kinematics code here ***/
        /*** Add a brief explanation of the algorithm you choose and any other design decisions you make ***/

        /** I chose the CCD algorithm, I followed the calculations for the axis and the angle as shown in the class slides. 
            For the FK part, I just reused a part of my implementation of the ForwardKinematics.cs file. I decided to use the root as the upper
            bound in the IK chain, so the IK chain will go all the way up to the hip. For IK iterations I choose 100 and for a threshold I chose
            chose 0.03. There's no real reason for choosing these, they just seemed to give decent enhough results. 
         **/

        /*** code to be completed by students begins ***/
        List<Joint> ikChain = new List<Joint>();        
        Joint cur = endEffector.GetParent(); 
        while(cur != null)
        {
            ikChain.Add(cur); 
            cur = cur.GetParent();            
        }
        

        // foreach (Joint curJ in ikChain)
        // {
        //     Debug.Log(curJ.Name); 
        // }
        // Debug.Log("--------"); 

        void oneCCDIteration(List<Joint> ikChain)
        {
            foreach(Joint curJoint in ikChain){
                Vector3 endDist = endEffector.GlobalPosition - curJoint.GlobalPosition;  
                Vector3 targetDist =  targetPosition - curJoint.GlobalPosition;  

                float rotAngle = Vector3.Dot(endDist.normalized, targetDist.normalized ); 
                rotAngle = Mathf.Clamp(rotAngle, -1f, 1f);
                rotAngle = Mathf.Acos(rotAngle); 
                rotAngle = rotAngle * Mathf.Rad2Deg; 

                Vector3 r = Vector3.Cross(endDist.normalized, targetDist.normalized); 
                if(r.magnitude < 0.0001f)
                {
                    continue; 
                }
                r = r.normalized;  

                Quaternion rotQuat = Quaternion.AngleAxis(rotAngle, r);
                Quaternion newGlobal = rotQuat * curJoint.GlobalQuaternion;  

                if(curJoint.GetParent() != null)
                {
                    curJoint.LocalQuaternion = Quaternion.Inverse(curJoint.GetParent().GlobalQuaternion) * newGlobal; 
                }
                else
                {
                    curJoint.LocalQuaternion = newGlobal; 
                }

                fkUpdates();

            }
        }

        const int ITERATIONS = 100; 
        const float THRESH = 0.03f; 
        for(int i = 0; i < ITERATIONS; i++)
        {
            oneCCDIteration(ikChain);  

            float actualDist = (endEffector.GlobalPosition - targetPosition).magnitude; 
            if(actualDist < THRESH)
            {
                break;  
            }            
        }

        fkUpdates();

        void fkUpdates()
        {
            foreach(Joint curNode in actor.Joints)
            {
                if(curNode.ParentIdx == -1)
                {
                    //dont actually need to do anything here, if you do update these then the position of model braks
                    //curNode.GlobalQuaternion = curNode.LocalQuaternion;
                    //curNode.GlobalPosition = curNode.LocalPosition; 
                }
                else if(curNode.RotateOrder != Joint.RotationOrder.NONE)
                {
                    curNode.GlobalQuaternion = curNode.GetParent().GlobalQuaternion * curNode.LocalQuaternion; 
                    curNode.GlobalPosition = curNode.GetParent().GlobalPosition + curNode.GetParent().GlobalQuaternion * curNode.LocalPosition; 
                }
                else
                {
                    // this path shouldn't occur anyways because during the ikChain step i didn't include end effectorss
                    curNode.GlobalQuaternion = curNode.GetParent().GlobalQuaternion * curNode.LocalQuaternion; 
                    curNode.GlobalPosition = curNode.GetParent().GlobalPosition + curNode.GetParent().GlobalQuaternion * curNode.LocalPosition; 
                }
            }
        }

        /*** code to be completed by students ends ***/
    }
}