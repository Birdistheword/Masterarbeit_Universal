using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightMove : MonoBehaviour
{
    [SerializeField] float minVal, maxVal;
    [SerializeField][Range(.1f, 7f)] float speed;
    private bool goingUp = true;
    bool hasChanged = false;
    private Vector3 targetPos;

 
    
    void Update()
    {
        if(goingUp)
        {
            // Go Up;
            //ChangeTargetPos(maxVal);
            transform.Translate(Vector3.up * Time.deltaTime * speed);
        }
        else if (!goingUp)
        {
            // Go Down;
            //ChangeTargetPos(minVal);
            transform.Translate(Vector3.down * Time.deltaTime * speed);
        }

        if(transform.position.y >= maxVal || transform.position.y <= minVal)
        {
            if (!hasChanged)
            {
                ChangeDirection();
                StartCoroutine(Counter());
            }
        }

        
    }

    private void ChangeDirection()
    {
        hasChanged = true;
        goingUp = !goingUp;
    }

    private IEnumerator Counter()
    {
        yield return new WaitForSeconds(1f);
        hasChanged = false;
    }
}
