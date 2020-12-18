using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveBetweenWaypoints : MonoBehaviour
{
    [SerializeField] GameObject objToMove;
    [SerializeField] Transform waypoint;
    [SerializeField] float moveSpeed = 0.5f;


   
    void FixedUpdate()
    {
        if(waypoint != null)
        {
            objToMove.transform.position = Vector3.Lerp(objToMove.transform.position, waypoint.position, moveSpeed * Time.deltaTime);
        }
    }
}
