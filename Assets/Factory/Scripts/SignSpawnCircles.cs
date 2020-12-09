using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SignSpawnCircles : MonoBehaviour
{
    [SerializeField] Image ball;
    [SerializeField] Transform[] Spawners;
    [SerializeField] Sprite greenSprite, whiteSprite;

    private Image currentImage;

        

    public void SpawnCircleInSection(int index)
    {
        Image newImage = Instantiate(ball, Spawners[index]);
        currentImage = newImage;
    }

    public void KillCircle()
    {
        Destroy(currentImage);
    }

    public void CorrectFillChangeImage()
    {
        currentImage.sprite = greenSprite;
    }

    public void EmptyChangeImage()
    {
        currentImage.sprite = whiteSprite;
    }

}
