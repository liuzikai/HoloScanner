using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using UnityEngine.UI;

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
    }

    private void TransitionState() {
        image.fillAmount = 0;
        if(state == State.Scanning) {
            currentRotationY = 0;
            image.color = stopColor;
            controller.StopScanning();
        } else {
            currentRotationY = 180;
            image.color = startColor;
            controller.StartScanning();
        }
        timer = 0;
        state = State.Transitioning;
    }

    void Update() {
        if(state == State.Transitioning) {
            timer += Time.deltaTime;
            transform.localRotation = Quaternion.Euler(0, currentRotationY + 180 * timer / TRANSITION_DURATION, 0);
            if(timer >= TRANSITION_DURATION) {
                timer = 0;
                state = controller.IsScanning ? State.Scanning : State.NotScanning;
            }
        } else {
            Ray ray = new Ray(gazeProvider.GazeOrigin, gazeProvider.GazeDirection);
            isGazing = Physics.Raycast(ray, 10, gazableMask);
            if (isGazing) {
                Debug.Log("Gazing!");
                timer += Time.deltaTime;
            } else {
                Debug.Log(ray);
                timer -= Time.deltaTime;
            }
            timer = Mathf.Clamp(timer, 0, FILL_DURATION);
            image.fillAmount = timer / FILL_DURATION;
            if (timer >= FILL_DURATION) {
                TransitionState();
            }
        }
    }
}
