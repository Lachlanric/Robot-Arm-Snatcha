using UnityEngine;
using System.IO.Ports;
using System.Linq;

public class serial_comm : MonoBehaviour
{
    public string portName = "COM5"; // Set the COM port (e.g., COM3)
    public int baudRate = 115200; // Set the baud rate
    private SerialPort serialPort;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Debug.Log("Hello World!");
        
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

        int[] dataToSend = { 30, 156, 142, 149 }; // Example integers in range 0-255
        SendBytes(dataToSend);
    }

    // Update is called once per frame
    void Update()
    {
        
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
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log($"Serial port {portName} closed.");
        }
    }
}
