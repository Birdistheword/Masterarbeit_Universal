using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoMaPlayer : MonoBehaviour
{
    [SerializeField] GameObject shaderPlane, cube1, LookPos;
    private ChangeMats cubeScript1;
    private SetImageShader setImgShader;
    private Vector3 screenStuff;
    private float maxD = 40f;
    


    void Start()
    {
        setImgShader = GameObject.FindObjectOfType<SetImageShader>();
        screenStuff = new Vector3(Screen.width / 2f, Screen.height / 2f, 0f);
        cubeScript1 = cube1.GetComponent<ChangeMats>();
    }

    // Update is called once per frame
    void LateUpdate()
    {
        // Layer 9 is the RaycastTrigger Layer. Only want to interact with objects here
        int layerMask = 1 << 9;
        RaycastHit hit;
        Vector3 fwd = transform.TransformDirection(Vector3.forward);
        Ray ray = new Ray(transform.position, fwd );

        Debug.DrawRay(transform.position, fwd * maxD, Color.red);

        if (Physics.Raycast(ray, out hit, maxD, layerMask)) {
            if(hit.collider.gameObject.name.Equals("ToggleShaderPlane"))
            {
                setImgShader.SetMultiplier(true);
            }
        }
    }

    public Vector3 GetOrig() 
    {
        return transform.position;
    }
    public Vector3 GetDir()
    {
        return transform.TransformDirection(Vector3.forward);
    }

}
