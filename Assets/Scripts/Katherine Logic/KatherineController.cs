using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KatherineController : MonoBehaviour
{
    [SerializeField] Animator anim;


    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ResetSpeed()
    {
        anim.SetFloat("hit_intensity", 0);
    }
}
