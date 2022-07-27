using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Waypoint : MonoBehaviour
{
    public int index;
    public float triggerDist = 1f;
    public float captureTime = 5f;
    private float captureStartTime = -1f;
    private bool reached = false;

    private void FixedUpdate() {
        Transform robotTf = GameManager.instance.robotTf;

        this.transform.position = new Vector3(this.transform.position.x, Mathf.Sin(Time.time) * 0.05F, this.transform.position.z);

        if (index == 0) {
            Destroy(this);
            return;
        }

        if (!reached && (robotTf.position - transform.position).sqrMagnitude < triggerDist * triggerDist) {
            if (captureStartTime < 0) {
                captureStartTime = Time.fixedTime;
                GameManager.instance.StartCapture();
            }
            
            if (captureStartTime + captureTime < Time.fixedTime) {
                reached = true;
                GameManager.instance.FinishCapture();
                GameManager.instance.ReachWaypoint(index);
                Destroy(this);
            }
        } else {
            if (captureStartTime > 0) {
                captureStartTime = -1f;
                GameManager.instance.EndCapture();
            }
        }
    }
}
