using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PickRandomAnim : MonoBehaviour
{
    [SerializeField] Animator thisAnimator;
    [SerializeField] AnimationClip[] clips;

    void Start()
    {
        int rnNumber = Random.Range(0, 1);


    }

    
}
