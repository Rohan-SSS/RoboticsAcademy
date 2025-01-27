import React, { useEffect, useRef, useState } from "react";

function decode_utf8(s) {
  return decodeURIComponent(escape(s));
}
const cameraErrorMessages = {
  NotAllowedError: "Camera access denied by the user.",
  OverconstrainedError:
    "The camera is already being used by another application or tab",
  NotFoundError: "No media devices found.",
  DevicesNotFoundError: "No media devices found.",
};

function Camera() {
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const videoRef = useRef(null);
  const streamRef = useRef(null); // Usamos useRef para manejar el stream
  const [stream, setStream] = useState(null);
  const [imageData, setImageData] = React.useState("");

  const [isCameraReady, setIsCameraReady] = useState(false);
  const [pauseCamera, setPauseCamera] = useState(false);
  const [isVisualReady, setIsVisualReady] = useState(false);
  const [error, setError] = useState("Connecting to media device.");

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

  // Obtener el stream de la cámara
  useEffect(() => {
    if (!isVisualReady) return;

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
            setIsCameraReady(true);
            setError([]);
            // Establecer el stream y asignarlo al video
            setStream(stream);
            if (videoRef.current) {
              videoRef.current.srcObject = stream;
              streamRef.current = stream; // Guardamos el stream en la referencia
            }
          })
          .catch((err) => {
            setIsCameraReady(false);

            const errorMessage = cameraErrorMessages[err.name];
            setError(errorMessage ? errorMessage : `Something went wrong!`);

            console.log(err);
          });
      }
    };

    startCamera();
    // Limpiar el stream cuando el componente se desmonte
    return () => {
      if ((streamRef, current)) {
        streamRef.getTracks().forEach((track) => track.stop());
      }
    };
  }, [isVisualReady]);

  // handle and udpate camera state, depending on RAM state
  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
        setIsVisualReady(true);
      }
      if (message.data.state === "application_running") {
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
    if (!isVisualReady || !isCameraReady) return;

    const callback = (message) => {
      if (message.data.update.image) {
        if (message.data.update.image) {
          let image_data = JSON.parse(message.data.update.image);
          let source = decode_utf8(image_data.image);

          if (source.length > 0)
            setImageData(`data:image/jpeg;base64,${source}`);
        }

        // Send the ACK of the msg
        //window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
      }
      // receive ack from gui.py
      if (message.data.update.ack_img === "ack" && !pauseCamera) {
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
  }, [pauseCamera, isVisualReady, isCameraReady]);

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        width: "100%",
        height: "100%",
      }}
    >
      {/* Contenedor de la primera imagen (video) */}
      <div
        style={{
          flex: 1,
          position: "relative",
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
        }}
      >
        {/* When get any error from webRtc Camera */}
        {!isCameraReady && (
          <div
            style={{
              position: "absolute",
              display: "flex",
              flexDirection: "column",
              gap: "10px",
              justifyContent: "center",
              alignItems: "center",
            }}
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              height="80px"
              viewBox="0 -960 960 960"
              width="80px"
              fill="#aaa"
            >
              <path d="M880-275 720-435v111l-60-60v-356H304l-60-60h416q24 0 42 18t18 42v215l160-160v410ZM848-27 39-836l42-42L890-69l-42 42ZM484-560Zm-87 82ZM159-800l60 60h-79v520h520v-79l60 60v19q0 24-18 42t-42 18H140q-24 0-42-18t-18-42v-520q0-24 18-42t42-18h19Z" />
            </svg>
            {error.length > 0 && <h3 style={{ color: "#FB2B36" }}>{error}</h3>}
          </div>
        )}
        <video
          ref={videoRef}
          autoPlay
          style={{
            width: "500px",
            height: "auto", // Mantener la proporción del video
            maxHeight: "100%", // Asegura que no se salga del contenedor
            objectFit: "contain", // Ajusta el video sin distorsionarlo
          }}
        />
      </div>

      {/* Contenedor flex-container con fondo blanco */}
      <div style={{ flex: "0 1 10px", backgroundColor: "#fff" }}>
        {/* Este es el contenedor intermedio que separa las dos imágenes, ahora con fondo blanco */}
      </div>

      {/* Contenedor de la segunda imagen */}
      <div
        style={{
          flex: 1,
          position: "relative",
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
        }}
      >
        {imageData && (
          <img
            src={imageData}
            alt="Imagen"
            style={{
              width: "500px",
              height: "auto", // Mantener la proporción de la imagen
              maxHeight: "100%", // Asegura que no se salga del contenedor
              objectFit: "contain", // Ajusta la imagen sin distorsionarla
            }}
          />
        )}
      </div>
    </div>
  );
}

export default Camera;
