using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetImageShader : MonoBehaviour
{
    [SerializeField] Material imageMat;
    [SerializeField] [Range(1f, 100f)] float decayRate = 10f, decayRateReversed = 30f;
    private float timer, effect = 0f;
    private int multiplier = 0;
    private bool multiply = false;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (multiply)
        {
            timer += Time.deltaTime / decayRate;
            effect = 1 - Mathf.Clamp(timer, 0f, 1f);
            imageMat.SetFloat("_IntensityEffect", effect);
            multiply = false;
        }

        else
        {
            timer -= Time.deltaTime / decayRateReversed;
            effect = 1 - Mathf.Clamp(timer, 0f, 1f);
            imageMat.SetFloat("_IntensityEffect", effect);
        }



    }

    public void SetMultiplier(bool set)
    {
        if (set)
        {
            multiply = true;
        }
    }

    void OnApplicationQuit()
    {
        imageMat.SetFloat("_IntensityEffect", 1f);
    }
}
