using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetShaderActive : MonoBehaviour
{
    [SerializeField] Material mat;

    
    public void ToggleShader(bool active)
    {
        if(active)
        {
            mat.SetFloat("_isActive", 1f);
        }
        else
        {
            mat.SetFloat("_isActive", 2f);
        }
    }
}
