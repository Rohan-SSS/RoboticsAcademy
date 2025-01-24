import React, { useEffect, useRef, useState } from "react";

function Camera() {
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const videoRef = useRef(null);
  const streamRef = useRef(null); // Usamos useRef para manejar el stream
  const [stream, setStream] = useState(null);
  const [readyCamera, setReadyCamera] = useState(false);
  const [pauseCamera, setPauseCamera] = useState(false);
  // Obtener el stream de la cámara
  useEffect(() => {
    if (!readyCamera) return;

    const startCamera = () => {
      const frameRate = 20;
      const constraints = {
        video: {
          frameRate: { ideal: frameRate, min: frameRate, max: frameRate },
        },
        audio: false,
      };
      if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
        navigator.mediaDevices
          .getUserMedia({ video: true, audio: false })
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
      canvas.width = 320;
      canvas.height = 240;

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

  const [isInterval, setIsInterval] = useState(true);
  // Llamar a captureFrame cada 100 ms para enviar imágenes de la cámara
  // useEffect(() => {
  //   if (readyCamera && !pauseCamera) {
  //     const interval = setInterval(() => {
  //       captureFrame();
  //     }, 0);
  //     return () => {
  //       clearInterval(interval);
  //     };
  //   }
  // }, [readyCamera, pauseCamera]);

  useEffect(() => {
    // message.data.state === "visualization_ready" ||
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
        setReadyCamera(true);
      }
      if (message.data.state === "application_running") {
        // setReadyCamera(true);
        setPauseCamera(false);
        captureFrame();
      } else if (message.data.state === "paused") {
        setPauseCamera(true);
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
      // console.log("message ", message.data.update.ack_img);
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
