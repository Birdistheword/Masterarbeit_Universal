using System.Collections.Generic;
using UnityEngine;

namespace HurricaneVR.Framework.Shared
{
    public static class HVRUtilities
    {
        public static Vector3 FindNearestPointOnLine(Vector3 origin, Vector3 end, Vector3 point)
        {
            //Get heading
            var heading = (end - origin);
            float magnitudeMax = heading.magnitude;
            heading.Normalize();

            //Do projection from the point but clamp it
            var lhs = point - origin;
            float dotP = Vector3.Dot(lhs, heading);
            dotP = Mathf.Clamp(dotP, 0f, magnitudeMax);
            return origin + heading * dotP;
        }
        
        public static Vector3 OrthogonalVector(Vector3 v)
        {
            //////https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector
            v.Normalize();
            var x = v.x;
            var y = v.y;
            var z = v.z;
            var v1 = new Vector3(0f, z, -y);
            var v2 = new Vector3(-z, 0f, x);
            var v3 = new Vector3(-y, x, 0f);
            var largest = v1;
            if (v2.magnitude > largest.magnitude)
                largest = v2;
            if (v3.magnitude > largest.magnitude)
                largest = v3;
            return largest;
        }
    }
}