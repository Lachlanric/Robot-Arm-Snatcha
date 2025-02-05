using UnityEngine;
using UnityEngine.XR;
using UnityEngine.InputSystem;

using System.IO.Ports;
using System.Linq;

public class Position_Tracker : MonoBehaviour
{
    public InputActionReference triggerAction;
    public InputActionReference gripAction;

    private bool triggered = false;
    private Vector3 triggerPosition;
    private Vector3 previousEndEffectorPosition = new Vector3(0,0,0);
    private Vector3 endEffectorPosition = new Vector3(0,0,0);
    private Vector3 lastGoodEndEffectorPosition = new Vector3(0,0,0);
    private float endEffectorAngle;
    private float claw_position;
    
    public string portName = "COM5"; // Set the COM port (e.g., COM3)
    public int baudRate = 115200; // Set the baud rate
    private SerialPort serialPort;

    private const int numMotors = 5;

    int[] dataToSend;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        triggerAction.action.Enable();
        gripAction.action.Enable();

        try
        {
            serialPort = new SerialPort(portName, baudRate)
            {
                ReadTimeout = 100, // Optional: Timeout for reading
                WriteTimeout = 100 // Optional: Timeout for writing
            };

            serialPort.Open();
            Debug.Log($"Serial port {portName} opened at {baudRate} baud.");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to open serial port {portName}: {e.Message}");
        }

        // SendBytes(dataToSend);
    }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log(transform.eulerAngles);
        // return;

        float triggerValue = triggerAction.action.ReadValue<float>();
        float gripValue = gripAction.action.ReadValue<float>();
        // Debug.Log(triggerValue);
        // Debug.Log(gripValue);
        if (gripValue==1) {

            if (!triggered) {
                triggered = true;
                triggerPosition = transform.position;
            }

            endEffectorPosition = previousEndEffectorPosition + transform.position - triggerPosition;
            endEffectorAngle = radians(transform.eulerAngles.x);

            if (endEffectorAngle>Mathf.PI) {endEffectorAngle-=2*Mathf.PI;}
            if (endEffectorAngle<-Mathf.PI) {endEffectorAngle+=2*Mathf.PI;}

            claw_position = radians(triggerValue * 255);

            dataToSend = calculateMotorVals(endEffectorPosition, endEffectorAngle, claw_position);
            if (dataToSend != null) {
                lastGoodEndEffectorPosition = endEffectorPosition;
                SendBytes(dataToSend);
            }

        } else {

            if (triggered) {
                triggered = false;
                previousEndEffectorPosition = lastGoodEndEffectorPosition;
            }
        }
    }



    public void SendBytes(int[] intList)
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                // Ensure all integers are within the range 0-255
                byte[] byteArray = intList.Select(i => (byte)i).ToArray();
                serialPort.Write(byteArray, 0, byteArray.Length);
                // Debug.Log("Sent bytes: " + BitConverter.ToString(byteArray));
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error sending data: {e.Message}");
            }
        }
        else
        {
            Debug.LogError("Serial port is not open.");
        }
    }

    private void OnApplicationQuit()
    {
        // Close the serial port when the application quits
        // if (serialPort != null && serialPort.IsOpen)
        // {
        serialPort.Close();
        Debug.Log($"Serial port {portName} closed.");
        // }
    }

    float[,,] motorAngleLerps = {
        {{90,74},  {180,219}},
        {{0, 20},  {90, 184 }},
        {{0, 25},  {180,246}},
        {{0, 92},  {90, 215 }},
        {{0, 255}, {255,0 }}
    };

    int angleToMotorVal(float ang, int motorIdx)
    {
        float x1,y1,x2,y2,m,motorVal;
        x1 = motorAngleLerps[motorIdx,0,0];
        y1 = motorAngleLerps[motorIdx,0,1];
        x2 = motorAngleLerps[motorIdx,1,0];
        y2 = motorAngleLerps[motorIdx,1,1];
        m = (y2-y1)/(x2-x1);

        motorVal = y1 + m*(ang - x1);
        return (int)motorVal;
    }

    float degrees(float radians) {
        return ((180.0f / Mathf.PI) * radians);
    }

    float radians(float degrees) {
        return ((Mathf.PI / 180.0f) * degrees);
    }

    int[] calculateMotorVals(Vector3 coords, float W, float C) {

        float X, Y, Z;
        X = - coords.x * 100;
        Y = coords.y * 100;
        Z = - coords.z * 100;

        // W = Mathf.Min(0, (Y/Mathf.PI)-Mathf.PI);

        float[] vals = {X,Y,Z,W};
        Debug.Log(vals.Aggregate("", (a,c)=>a + ", " + c.ToString()));

        float L = 10, h = 10;
        float R, ang, q, alpha, theta, phi, psi;

        R = Mathf.Sqrt(Mathf.Pow(X,2) + Mathf.Pow(Z,2));
        ang = Mathf.Atan2(Z,X);

        q = Mathf.Sqrt(Mathf.Pow(R,2) + Mathf.Pow((Y-h),2));
        alpha = Mathf.Atan2((Y-h),R);
        theta = Mathf.Acos(q/(2*L)) + alpha;

        phi = Mathf.PI - 2*(theta-alpha);
        psi = -(theta - Mathf.PI + phi) + W;

        float[] motorAngles = {phi, theta, ang, psi, C};  // in radians
        int[] motorVals = {-1,-1,-1,-1,-1};

        // Convert angles to motor vals
        for(int i=0; i<numMotors; i++) {
            motorVals[i] = (int)(angleToMotorVal(degrees(motorAngles[i]),i));
        }
        
        // Clamp wrist angle
        motorVals[3] = Mathf.Clamp(motorVals[3],0,255);

        // Detect invalid motor angles
        for(int i=0; i<numMotors; i++) {
            if (motorVals[i]<0 || motorVals[i]>255) {
                // Debug.LogWarning("Motor values out of range:");
                Debug.LogWarning("Motor values out of range:" + motorVals.Aggregate("", (a,c)=>a + ", " + c.ToString()));
                return null;
            }
        }
        
        return motorVals;
    }


}
