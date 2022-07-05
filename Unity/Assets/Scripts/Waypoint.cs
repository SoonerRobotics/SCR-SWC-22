using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Waypoint : MonoBehaviour
{
    public int index;
    public float triggerDist = 1f;
    private bool reached = false;

    private void FixedUpdate() {
        Transform robotTf = GameManager.instance.robotTf;

        this.transform.position = new Vector3(this.transform.position.x, Mathf.Sin(Time.time) * 0.05F, this.transform.position.z);

        if (index == 0) {
            Destroy(this);
            return;
        }

        if (!reached && (robotTf.position - transform.position).sqrMagnitude < triggerDist * triggerDist) {
            reached = true;
            GameManager.instance.ReachWaypoint(index);
            Destroy(this);
        }
    }
}
