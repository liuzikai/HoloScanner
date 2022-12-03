using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StopStartController : MonoBehaviour
{
    public bool IsScanning { get; private set; }

    public void StartScanning() {
        IsScanning = true;
        Debug.Log("Send Start signal!");
        //TODO call internal 'Start Scanning' Binding
    }

    public void StopScanning() {
        IsScanning = false;
        Debug.Log("Send Stop signal!");
        //TODO call internal 'Stop Scanning' Binding
    }
}
