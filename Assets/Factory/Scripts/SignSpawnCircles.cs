using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SignSpawnCircles : MonoBehaviour
{
    [SerializeField] Image ball;
    [SerializeField] Transform[] Spawners;
    [SerializeField] Sprite greenSprite, whiteSprite;

    private List<Image> currentImageRow = new List<Image>();
    private int currentImageIndex = 0;
    private Image currentImage;
    //private Vector2 offset = new Vector2(0.3f, 0);

        

    public void SpawnCircleInSection(int index, int amount)
    {
        for(int i = 0; i < amount; i++)
        {
            Image newImage = Instantiate(ball, Spawners[index]);

            // Move a bit
            newImage.transform.localPosition = new Vector3(0.25f * i, 0, 0);
            currentImageRow.Add(newImage);
            
        }
        currentImage = currentImageRow[0];

    }
    
    
    public void CorrectFillChangeImage()
    {
        currentImage.sprite = greenSprite;

        if(currentImageIndex<currentImageRow.Count-1)
        {
            currentImageIndex++;
            currentImage = currentImageRow[currentImageIndex];
        }
        
    }

    public void EmptyChangeImage()
    {
        if(currentImageIndex > 0)
        {
            currentImageIndex--;
            currentImage = currentImageRow[currentImageIndex];
        }
        
        currentImage.sprite = whiteSprite;
    }

    public void KillCircle()
    {

        foreach (Image img in currentImageRow)
        {
            Destroy(img);
        }

        currentImageRow.Clear();
    }

}
