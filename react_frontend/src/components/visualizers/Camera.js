import React, { useEffect, useRef, useState } from "react";

function Camera() {
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const videoRef = useRef(null);
  const streamRef = useRef(null); // Usamos useRef para manejar el stream
  const [stream, setStream] = useState(null);
  const [readyCamera, setReadyCamera] = useState(false);
  const [pauseCamera, setPauseCamera] = useState(false);

  // count fps
  const [frameRates, setFrameRates] = useState(0);
  const [lastTime, setLastTime] = useState(performance.now());
  // Obtener el stream de la cámara
  useEffect(() => {
    if (!readyCamera) return;

    const startCamera = () => {
      // configure media parameters
      const constraints = {
        video: {
          width: { min: 320, ideal: 320, max: 320 },
          height: { min: 240, ideal: 240, max: 240 },
          frameRate: { ideal: 30, max: 30 },
        },
        audio: false,
      };
      if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
        navigator.mediaDevices
          .getUserMedia(constraints)
          .then((stream) => {
            // Establecer el stream y asignarlo al video
            setStream(stream);
            if (videoRef.current) {
              videoRef.current.srcObject = stream;
              streamRef.current = stream; // Guardamos el stream en la referencia
            }
          })
          .catch((err) => console.log(err));
      }
    };

    startCamera();
    // Limpiar el stream cuando el componente se desmonte
    return () => {
      if ((streamRef, current)) {
        streamRef.getTracks().forEach((track) => track.stop());
      }
    };
  }, [readyCamera]);

  // Función para capturar un fotograma del video y convertirlo en una matriz CV_8UC4
  const captureFrame = () => {
    const video = videoRef.current;
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");

    if (video && canvas && ctx) {
      // Establecer el tamaño del canvas igual al tamaño del video
      canvas.width = 320; //320;
      canvas.height = 240; //240;

      // Dibujar el frame del video en el canvas
      ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

      // Obtener los datos de la imagen (array de píxeles RGBA)
      const imageDataURL = canvas.toDataURL("image/jpeg");
      // Codificamos en base64
      // Enviar la matriz por WebSocket
      window.RoboticsExerciseComponents.commsManager.send(
        "gui",
        `pick${imageDataURL}`
      );
    }
  };

  useEffect(() => {
    // message.data.state === "visualization_ready" ||
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
        setReadyCamera(true);
      }
      if (message.data.state === "application_running") {
        setPauseCamera(false);
        captureFrame();
      } else if (message.data.state === "paused") {
        setPauseCamera(true);
        setFrameRates(0);
      }
    };
    commsManager.subscribe([commsManager.events.STATE_CHANGED], callback);

    return () => {
      commsManager.unsubscribe([commsManager.events.STATE_CHANGED], callback);
    };
  }, []);

  // ack (you can get response from update_gui() in GUI.py)
  useEffect(() => {
    const callback = (message) => {
      // receive ack from gui.py
      if (message.data.update.ack_img === "ack") {
        captureFrame();
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  return (
    <div style={{ display: "flex", width: "100%", height: "100%" }}>
      <video ref={videoRef} autoPlay></video>
    </div>
  );
}

export default Camera;
