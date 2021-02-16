using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoMaPlayer : MonoBehaviour
{
    [SerializeField] GameObject shaderPlane;
    private SetShaderActive setShader;


    void Start()
    {
        setShader = GameObject.FindObjectOfType<SetShaderActive>();
    }

    // Update is called once per frame
    void Update()
    {
        RaycastHit hit;
        Ray ray = new Ray(transform.position, Vector3.forward);

        if (Physics.Raycast(ray, out hit))
        {
            Transform objectHit = hit.transform;

            // Do something with the object that was hit by the raycast.
            print(hit.collider.gameObject.name);
        }
    }
}
