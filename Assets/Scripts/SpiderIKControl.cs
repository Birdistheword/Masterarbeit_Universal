using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpiderIKControl : MonoBehaviour
{
    [SerializeField] GameObject FootIK, FootIKStart;
    [SerializeField] Transform BodyTarget;
    [SerializeField][Range(5f, 20f)] float legSpeed = 15f;
    [SerializeField] [Range(.1f, 20f)] float legParabolaHeight = 0.8f;
    [SerializeField][Range(0.5f, 2f)] float stepRange= 0.8f;

    [SerializeField] private bool legCanMove = false, isStepping = false;

    private Vector3 footStartPos, bodyTargetOldPos, dir, Offset;
    private float anim;


    void Start()
    {
        Offset = new Vector3(0f, 0.523f, 0f);
        FootIKStart.transform.position = FootIK.transform.position;
    }

    void FixedUpdate()
    {


        if (Vector3.Distance(BodyTarget.position+ Offset, FootIKStart.transform.position) >= stepRange)
        {
            bodyTargetOldPos = BodyTarget.position;
            dir = (bodyTargetOldPos + Offset)- FootIKStart.transform.position;
            legCanMove = true;
            isStepping = true;
        }

        anim += Time.deltaTime;

        anim = anim % 1f;

        if(legCanMove)
        {
            if (Vector3.Distance(bodyTargetOldPos + Offset, FootIKStart.transform.position) <= .05f)
            {
                legCanMove = false;
                return;
            }
            //FootIKStart.transform.Translate(dir * Time.deltaTime * legSpeed);
            FootIKStart.transform.position = MathParabola.Parabola(FootIKStart.transform.position,
                                                                    bodyTargetOldPos + Offset,
                                                                    legParabolaHeight,
                                                                    anim / 1f);
        }

        FootIK.transform.position = FootIKStart.transform.position;
    }
}
