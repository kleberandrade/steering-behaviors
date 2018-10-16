using UnityEngine;

public class Boid : MonoBehaviour
{
    public Vector3 m_Direction;

    public float m_MaxSpeed;

    private void FixedUpdate()
    {
        transform.Translate(Velocity);
    }

    public Vector3 Velocity
    {
        get
        {
            return m_Direction.normalized * m_MaxSpeed * Time.fixedDeltaTime;
        }
    }

    public Vector3 Position
    {
        get
        {
            return transform.position;
        }

        set
        {
            transform.position = value;
        }
    }
}
