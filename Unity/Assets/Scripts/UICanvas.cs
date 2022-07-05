using UnityEngine;

public class UICanvas : MonoBehaviour
{
    public GameObject PauseMenu;
    public GameObject RecordMenu;
    public GameObject Slider;
    public GameObject SliderText;

    public bool paused = false;
    public int targetFreq = 735;

    public int minFreq = 735;
    public int maxFreq = 792;

    public float timeToTarget = 5f;

    void Update() {
        if (Input.GetKeyDown(KeyCode.Escape)) {
            paused = !paused;
            PauseMenu.SetActive(paused);
        }

        float centerFreq = (maxFreq + minFreq) / 2f;

        // :)
        float scaledTime = Time.time % 7;
        if (scaledTime > timeToTarget) {
            scaledTime = timeToTarget;
        }
        float width = (maxFreq - minFreq) * (timeToTarget - scaledTime) / timeToTarget;
        float center = ((timeToTarget - scaledTime) * centerFreq + scaledTime * targetFreq) / timeToTarget;
        float freq = center + width * Mathf.Sin(Time.time * 3);

        RectTransform myRectTransform = Slider.GetComponent<RectTransform>();
        myRectTransform.localPosition = new Vector3((freq - centerFreq) / (maxFreq - minFreq) * 140, myRectTransform.localPosition.y, myRectTransform.localPosition.z);

        SliderText.GetComponent<TMPro.TextMeshProUGUI>().text = string.Format("{0:0} MHz", freq);
    }

    public void Restart() {
        GameManager.instance.RestartSim();
    }

    public void Reload() {
        GameManager.instance.ReloadSim();
    }
}
