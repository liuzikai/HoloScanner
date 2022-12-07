using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit;

public class GazeButton : MonoBehaviour {

    public GazeProvider gazeProvider;
    public LayerMask gazableMask;
    public Image image;
    public Color startColor;
    public Color stopColor;

    private const float FILL_DURATION = 2;
    private const float TRANSITION_DURATION = 1.2f;

    private bool isGazing = false;
    private float timer = 0;
    private StopStartController controller;
    private float currentRotationY = 0;

    enum State {
        Scanning,
        NotScanning,
        Transitioning,
    }
    private State state = State.NotScanning;

    void Start() {
        controller = GetComponentInParent<StopStartController>();
        image.color = startColor;
    }

    private void TransitionState() {
        image.fillAmount = 0;
        if(state == State.Scanning) {
            currentRotationY = 180;
            image.color = startColor;
            controller.StopScanning();
        } else {
            currentRotationY = 0;
            image.color = stopColor;
            controller.StartScanning();
        }
        timer = 0;
        state = State.Transitioning;
    }

    void Update() {
        // Debug.Log($"State: {state}");
        if (state == State.Transitioning) {
            timer += Time.deltaTime;
            transform.localRotation = Quaternion.Euler(0, currentRotationY + 180 * timer / TRANSITION_DURATION, 0);
            if(timer >= TRANSITION_DURATION) {
                timer = 0;
                state = controller.IsScanning ? State.Scanning : State.NotScanning;
            }
        } else {
            Ray ray = new Ray(CoreServices.InputSystem.GazeProvider.GazeOrigin, CoreServices.InputSystem.GazeProvider.GazeDirection);
            isGazing = Physics.Raycast(ray, 10, gazableMask);
            if (isGazing) {
                //Debug.Log("Gazing!");
                timer += Time.deltaTime;
            } else {
                timer -= Time.deltaTime;
            }
            // Debug.Log(ray);
            timer = Mathf.Clamp(timer, 0, FILL_DURATION);
            image.fillAmount = timer / FILL_DURATION;
            if (timer >= FILL_DURATION) {
                TransitionState();
            }
        }
    }
}
