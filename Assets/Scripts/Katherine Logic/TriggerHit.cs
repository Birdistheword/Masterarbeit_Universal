using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TriggerHit : MonoBehaviour
{
    [SerializeField] Animator anim;
    [SerializeField] AudioSource audio;
    [SerializeField] string hitText;

    private float speedOfHittingObj;

   
   
    private void OnTriggerEnter(Collider obj)
    {
        
        if (obj.GetComponent<Rigidbody>() != null)
        {
            Debug.Log("I got hit in " + hitText);
            speedOfHittingObj = obj.GetComponent<Rigidbody>().velocity.magnitude;
            Debug.Log("speed was: " + speedOfHittingObj);

            anim.SetFloat("hit_intensity", speedOfHittingObj);

            audio.Play();
        }
    }

   
}
