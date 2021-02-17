using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class ChangeMats : MonoBehaviour
{
    private Material[] allDissolveMats;
    public Transform lookPos;
    public MoMaPlayer playerScript;
    private Vector3 distance, orig, dir, nearestPoint;
    private float dist;

    void Start ()
    {
        allDissolveMats = GetComponentInChildren<MeshRenderer>().materials;
        playerScript = GameObject.FindObjectOfType<MoMaPlayer>();

        foreach (Material mat in allDissolveMats)
        {
            float x = Random.Range(0f, 3f);
            float y = Random.Range(7f, 15f);
            

            mat.SetVector("DistanceRemapRange_", new Vector2(x, y));
        }
    }

    void Update()
    {
        
        // Set Vectors according to player
        orig = playerScript.GetOrig();
        dir = playerScript.GetDir();

        nearestPoint = FindNearestPointOnLine(orig, dir, transform.position);
        DistanceToLook();
        SetValues();

    }

    private void DistanceToLook()
    {
        dist = Vector3.Distance(nearestPoint, transform.position);
    }

    public Vector3 FindNearestPointOnLine(Vector3 origin, Vector3 direction, Vector3 point)
    {
        direction.Normalize();
        Vector3 lhs = point - origin;

        float dotP = Vector3.Dot(lhs, direction);
        return origin + direction * dotP;
    }
    


    private void SetValues()
    {
        foreach(Material mat in allDissolveMats)
        {
            mat.SetFloat("activateEffect_", dist);
        }
    }

}
