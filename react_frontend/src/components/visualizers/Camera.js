import React, { useEffect, useRef, useState } from "react";

function decode_utf8(s) {
    return decodeURIComponent(escape(s));
}


function Camera() {
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const videoRef = useRef(null);
  const streamRef = useRef(null); // Usamos useRef para manejar el stream
  const [stream, setStream] = useState(null);
  const [readyCamera, setReadyCamera] = useState(false);
  const [pauseCamera, setPauseCamera] = useState(false);
  const [image, setImage] = React.useState(null);
  const [imageData, setImageData] = React.useState("");
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
  <div style={{ display: "flex", flexDirection: "column", width: "100%", height: "100%" }}>
    {/* Contenedor de la primera imagen (video) */}
    <div style={{ flex: 1, position: "relative", display: "flex", justifyContent: "center", alignItems: "center" }}>
      <video
        ref={videoRef}
        autoPlay
        style={{
          width: "500px",
          height: "auto", // Mantener la proporción del video
          maxHeight: "100%", // Asegura que no se salga del contenedor
          objectFit: "contain" // Ajusta el video sin distorsionarlo
        }}
      />
    </div>
    
    {/* Contenedor flex-container con fondo blanco */}
    <div style={{ flex: "0 1 10px", backgroundColor: "#fff" }}>
      {/* Este es el contenedor intermedio que separa las dos imágenes, ahora con fondo blanco */}
    </div>
    
    {/* Contenedor de la segunda imagen */}
    <div style={{ flex: 1, position: "relative", display: "flex", justifyContent: "center", alignItems: "center" }}>
      {imageData && (
        <img
          src={imageData}
          alt="Imagen"
          style={{
            width: "500px",
            height: "auto", // Mantener la proporción de la imagen
            maxHeight: "100%", // Asegura que no se salga del contenedor
            objectFit: "contain" // Ajusta la imagen sin distorsionarla
          }}
        />
      )}
    </div>
  </div>
);
}

export default Camera;
