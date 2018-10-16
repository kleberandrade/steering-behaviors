using System.Collections.Generic;
using UnityEngine;

public class SteeringBehaviors : MonoBehaviour
{
    [Header("Rigidbody")]
    public float m_Mass;
    public float m_MaxSpeed;
    public float m_MaxForce;
    private Vector3 m_CurrentVelocity;

    [Header("Seek and Flee")]
    public SteeringBehaviors m_Boid;
    public float m_SlowingRadius;

    [Header("Pursuit and Evade")]
    public int m_UpdateAhead;

    [Header("Path Following")]
    public Path m_Path;

    [Header("Wander Random")]
    public float m_Interval;
    public float m_Range;
    private float m_NextDecisionTime;

    [Header("Wander")]
    public float m_CircleDistance;
    public float m_CircleRadius;
    private float m_WanderAngle;
    public float m_WanderAngleChange;

    [Header("Collision Avoidance")]
    public float m_AvoidanceForce;
    public float m_MaxSeeAhead;
    public LayerMask m_ObstacleLayer;

    [Header("Flocking")]
    public string m_FlockingTag;
    public float m_NeighborDistance = 3.0f;
    public float m_VelocityVariation = 0.5f;
    public SphereCollider m_Collider;
    private List<GameObject> m_Flock = new List<GameObject>();
    private float m_NoiseOffset;

    [Header("Tank")]
    public float m_TankRadius = 20.0f;

    private void Awake()
    {
        m_Collider = GetComponent<SphereCollider>();
        m_Animation = GetComponent<Animation>();
    }


    private void Start()
    {
        m_CurrentVelocity = transform.forward * (m_MaxSpeed * Time.fixedDeltaTime);
        m_NextDecisionTime = Time.time + m_Interval;
    }

    private void FixedUpdate()
    {
        Vector3 steeringForce = Seek(m_Boid.Position, m_SlowingRadius);
        steeringForce += CollisionAvoidance(0.0f);
        steeringForce += CollisionAvoidance(-20.0f);
        steeringForce += CollisionAvoidance(20.0f);
        steeringForce += CollisionAvoidance(-45.0f);
        steeringForce += CollisionAvoidance(45.0f);

        steeringForce = steeringForce.normalized * (m_MaxForce * Time.fixedDeltaTime);

        Vector3 acceleration = steeringForce / m_Mass;
        m_CurrentVelocity += acceleration;
        transform.position += m_CurrentVelocity;

        transform.rotation = Quaternion.LookRotation(m_CurrentVelocity);
    }

    public Vector3 Seek(Vector3 target, float slowingRadius)
    {
        Vector3 desiredVelocity = target - transform.position;

        float distance = desiredVelocity.magnitude;
        if (distance <= m_SlowingRadius)
            desiredVelocity = desiredVelocity.normalized * (m_MaxSpeed * Time.fixedDeltaTime) * (distance / slowingRadius);
        else
            desiredVelocity = desiredVelocity.normalized * (m_MaxSpeed * Time.fixedDeltaTime);

        Vector3 steeringForce = desiredVelocity - m_CurrentVelocity;
        return steeringForce;
    }

    public Vector3 Flee(Vector3 target)
    {
        float speed = m_MaxSpeed * Time.fixedDeltaTime;

        Vector3 desiredVelocity = transform.position - target;
        desiredVelocity = desiredVelocity.normalized * speed;

        Vector3 steeringForce = desiredVelocity - m_CurrentVelocity;
        return steeringForce;
    }

    public Vector3 Pursuit(Boid boid, float slowingRadius)
    {
        Vector3 distance = boid.Position - transform.position;
        float updatesAhead = distance.magnitude / m_MaxSpeed;

        Vector3 futurePosition = boid.Position + boid.Velocity * updatesAhead * m_UpdateAhead;
        return Seek(futurePosition, slowingRadius);
    }

    public Vector3 Evade(Boid boid)
    {
        Vector3 distance = boid.Position - transform.position;
        float updatesAhead = distance.magnitude / m_MaxSpeed;

        Vector3 futurePosition = boid.Position + boid.Velocity * updatesAhead * m_UpdateAhead;
        return Flee(futurePosition);
    }

    public Vector3 WanderRandom()
    {
        float now = Time.time;
        if (now >= m_NextDecisionTime)
        {
            m_NextDecisionTime += m_Interval;

            Vector3 position = Vector3.zero;
            position.x = Random.Range(-m_Range, m_Range);
            position.z = Random.Range(-m_Range, m_Range);

            m_Boid.Position = position;
        }

        return Seek(m_Boid.Position, m_SlowingRadius);
    }

    public Vector3 PathFollow()
    {
        Vector3? target = m_Path.GetNode();

        if (Vector3.Distance(transform.position, (Vector3)target) <= m_Path.m_Radius)
            m_Path.NextNode();

        return Seek((Vector3)target, m_SlowingRadius);
    }

    public Vector3 Wander()
    {
        Vector3 circleCenter = transform.position + m_CurrentVelocity.normalized * m_CircleDistance;

        m_WanderAngle += Random.Range(-m_WanderAngleChange, m_WanderAngleChange);

        Vector3 displacement = Vector3.forward;
        displacement.x = m_CircleDistance * Mathf.Cos(m_WanderAngle * Mathf.Deg2Rad);
        displacement.z = m_CircleDistance * Mathf.Sin(m_WanderAngle * Mathf.Deg2Rad);

        Vector3 target = circleCenter + displacement;

        return Seek(target, m_SlowingRadius);
    }

    public Vector3 CollisionAvoidance(float angle)
    {
        RaycastHit hitInfo;

        Vector3 direction = Quaternion.AngleAxis(angle, transform.up) * transform.forward;

        if (Physics.Raycast(transform.position, direction, out hitInfo, m_MaxSeeAhead, m_ObstacleLayer))
        {
            Debug.DrawLine(transform.position, hitInfo.point, Color.red);
            return hitInfo.normal * ((1.0f - (hitInfo.distance / m_MaxSeeAhead)) * m_AvoidanceForce);
        }
        else
        {
            Debug.DrawLine(transform.position, transform.position + direction * m_MaxSeeAhead, Color.blue);
        }

        return Vector3.zero;
    }

    public Vector3 Position
    {
        get { return transform.position; }
        set { transform.position = value; }
    }

    public Vector3 Velocity
    {
        get { return m_CurrentVelocity; }
        set { m_CurrentVelocity = value; }
    }

    private void ApplyRules()
    {
        Vector3 currentPosition = transform.position;
        Quaternion currentRotation = transform.rotation;

        float noise = Mathf.PerlinNoise(Time.time, m_NoiseOffset) * 2.0f - 1.0f;
        float currentVelocity = m_CurrentVelocity * (1.0f + noise * m_VelocityVariation);

        if (VerifyTankBoundary(m_TankRadius))
        {
            Vector3 direction = Vector3.zero - transform.position;
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(direction), currentVelocity * Time.deltaTime);
        }
        else
        {
            Vector3 separation = Separation();
            Vector3 alignment = Alignment();
            Vector3 cohesion = Cohesion();

            cohesion = (cohesion - currentPosition).normalized;

            Vector3 direction = separation + alignment + cohesion;
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, direction.normalized);

            if (rotation != currentRotation)
            {
                float ip = Mathf.Exp(-m_RotationCoeff * Time.deltaTime);
                transform.rotation = Quaternion.Slerp(rotation, currentRotation, ip);
            }
        }

        transform.position = currentPosition + transform.forward * (currentVelocity * Time.deltaTime);
        m_Animation["Motion"].speed = currentVelocity;
    }

    private Vector3 Separation()
    {
        Vector3 separation = Vector3.zero;

        for (int i = 0; i < m_Flock.Count; i++)
        {
            Vector3 diff = transform.position - m_Flock[i].transform.position;
            float length = diff.magnitude;
            float scaler = Mathf.Clamp01(1.0f - length / m_NeighborDistance);
            separation += diff * (scaler / length);
        }

        return separation;
    }

    private Vector3 Cohesion()
    {
        Vector3 cohesion = transform.position;

        for (int i = 0; i < m_Flock.Count; i++)
            cohesion += m_Flock[i].transform.position;

        float average = 1.0f / m_Flock.Count;

        return cohesion * average;
    }

    private Vector3 Alignment()
    {
        Vector3 alignment = transform.forward;

        for (int i = 0; i < m_Flock.Count; i++)
            alignment += m_Flock[i].transform.forward;

        float average = 1.0f / m_Flock.Count;

        return alignment * average;
    }

    private bool VerifyTankBoundary(float radius)
    {
        if (Vector3.Distance(transform.position, Vector3.zero) >= radius)
            return true;

        return false;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag(m_FlockingTag))
        {
            m_Flock.Add(other.gameObject);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag(m_FlockingTag))
        {
            m_Flock.Remove(other.gameObject);
        }
    }
}
