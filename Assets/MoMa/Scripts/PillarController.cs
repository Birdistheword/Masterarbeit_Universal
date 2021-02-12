using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PillarController : MonoBehaviour
{
    [SerializeField] float speed;
    private Vector3 changeVector;
    private bool rising = true;
    private float minScale, maxScale;

    void Start()
    {
        speed = Random.Range(2f, 4f);
        
        minScale = Random.Range(1f, 3f);
        maxScale = Random.Range(9f, 12f);
        changeVector = new Vector3(0f, speed, 0f);
        transform.localScale = new Vector3(transform.localScale.x, minScale, transform.localScale.z);
        rising = maxScale >= 6f;

    }

    
    void Update()
    {
        if(rising)
        {
            transform.localScale += changeVector * Time.deltaTime;
            if (transform.localScale.y >= maxScale)
            {
                rising = false;
                
            }
        }
            
        else if (!rising)
        {
            transform.localScale -= changeVector * Time.deltaTime;
            if (transform.localScale.y <= minScale)
            {
                rising = true;
                ChangeValues();
            }
        }
            
    }

    private void ChangeValues()
    {
        minScale = Random.Range(1f, 3f);
        maxScale = Random.Range(9f, 12f);
        speed = Random.Range(2f, 4f);
        changeVector = new Vector3(0f, speed, 0f);
    }
}
