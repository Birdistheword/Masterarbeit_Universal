using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateThis : MonoBehaviour
{
    [SerializeField] float speedY = 1f;
    private Vector3 rotEuler;

    void Start()
    {
        //rotEuler = new Vector3(0, speed, speed);
    }

    void Update()
    {

        transform.Rotate(0f, speedY, 0f, Space.Self );
    }
}
