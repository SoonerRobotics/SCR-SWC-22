using UnityEngine;

public class UICanvas : MonoBehaviour
{
    public GameObject PauseMenu;
    public GameObject RecordMenu;
    public GameObject Slider;
    public GameObject SliderText;
    public GameObject RecordText;

    public bool paused = false;
    public float captureStartTime = -1f;
    public int targetFreq = 750;

    // :)
    public int minFreq = 735;
    public int maxFreq = 792;

    public float timeToTarget = 5f;
    public float timeToComplete = 7f;

    void Update() {
        if (Input.GetKeyDown(KeyCode.Escape)) {
            paused = !paused;
            PauseMenu.SetActive(paused);
        }

        if (GameManager.instance.CaptureStatus() == GameManager.CaptureState.CAPTURING && captureStartTime < 0f) {
            captureStartTime = Time.fixedTime;
            targetFreq = Random.Range(minFreq, maxFreq+1);
            RecordText.GetComponent<TMPro.TextMeshProUGUI>().text = string.Format("Recording RF...");
            RecordMenu.SetActive(true);
        }

        if (GameManager.instance.CaptureStatus() == GameManager.CaptureState.NONE) {
            captureStartTime = -1f;
            RecordMenu.SetActive(false);
            return;
        }

        float centerFreq = (maxFreq + minFreq) / 2f;

        float scaledTime = Time.fixedTime - captureStartTime;

        if (scaledTime > timeToComplete) {
            GameManager.instance.EndCapture();
            captureStartTime = -1f;
            RecordMenu.SetActive(false);
            return;
        }

        if (scaledTime > timeToTarget) {
            scaledTime = timeToTarget;
            RecordText.GetComponent<TMPro.TextMeshProUGUI>().text = string.Format("Peak RF detected at {0:0} MHz!", targetFreq);
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
