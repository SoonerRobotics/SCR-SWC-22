using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoatController : MonoBehaviour
{
    public GameObject centerOfMass;
    public GameObject robotCamera;
    public GameObject rudderVisual;

    private float turnDrag = 0.8f;
    [Range(20.0F, 100.0F)]
    public float boatMass = 50;
    [Range(20.0F, 500.0F)]
    public float maxThrust = 100;
    public float maxAccel = 1.0f;
    private float I_z = 50;
    private float D_u = 10;
    private float D_v = 200;
    private float D_rud = 1;
    private float M_rudder = 5;

    public float Angle { get; private set; }
    public float Power { get; private set; }

    public float CntrlAngle { get; private set; } = 0;
    public float CntrlPower { get; private set; } = 0;

    public Vector3 linear_vel { get; private set; } = new Vector3();
    public Vector3 angular_vel { get; private set; } = new Vector3();
    public Vector3 accel { get; private set; } = new Vector3();

    private float Radius;

    private Rigidbody rb;

    // Start is called before the first frame update
    void Start()
    {
        GameManager.instance.robotTf = centerOfMass.transform;

        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0,0,0);
        rb.inertiaTensor = new Vector3(100,100,100);
        rb.inertiaTensorRotation = Quaternion.identity;
        rb.maxAngularVelocity = 100f;

        maxAccel *= Time.fixedDeltaTime;
    }

    // Update is called once per frame
    void Update()
    {
        CntrlPower = Input.GetAxis("Speed");
        CntrlAngle = Input.GetAxis("Angle") * 20;
    }

    public void SetControl(float rpm, float angle) {
        CntrlPower = rpm / 5000;
        CntrlAngle = angle;
    }

    private void FixedUpdate()
    {

        Angle += (1.0f - turnDrag) * (CntrlAngle - Angle);
        Power += Mathf.Clamp(CntrlPower - Power, -maxAccel, maxAccel);

        Angle = Mathf.Clamp(Angle, -30, 30);
        Power = Mathf.Clamp(Power, -1, 1);

        float u = linear_vel.x;
        float v = linear_vel.z;
        float rudder = Angle * Mathf.Deg2Rad;
        float D_u_rudder = D_rud * Mathf.Cos(rudder);
        float D_v_rudder = D_rud * Mathf.Sin(rudder);

        // Rudder moment
        float tau_rudder = M_rudder * u * Mathf.Abs(u) * rudder;

        float u_drag = D_u * Mathf.Abs(u) * u + D_u_rudder * Mathf.Abs(u) * u;
        float v_drag = D_v * Mathf.Abs(v) * v + D_v_rudder * Mathf.Abs(u) * u;

        float thrust = maxThrust * Power;

        float u_dot = (1/boatMass) * (thrust - u_drag);
        float v_dot = (1/boatMass) * (-v_drag - tau_rudder);
        float r_dot = (1/I_z) * tau_rudder;

        accel = new Vector3(u_dot, 0, v_dot);
        linear_vel = new Vector3(linear_vel.x + u_dot, 0, linear_vel.z + v_dot);
        angular_vel = new Vector3(0, angular_vel.y + r_dot, 0);

        float heading = -this.transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        rb.velocity = new Vector3(linear_vel.x * Mathf.Cos(heading) + linear_vel.z * Mathf.Sin(heading), 0, linear_vel.x * Mathf.Sin(heading) + linear_vel.z * Mathf.Cos(heading));
        rb.angularVelocity = angular_vel;

        rudderVisual.transform.localRotation = Quaternion.Euler(new Vector3(0, Angle, 0));
    }
}
