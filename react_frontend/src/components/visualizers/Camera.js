import React, { useEffect, useRef } from 'react';
var ws_manager;
function Camera() {
  const videoRef = useRef(null);
  ws_manager = new WebSocket("ws://" + "127.0.0.1" + ":8765/");
  useEffect(() => {
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
      navigator.mediaDevices.getUserMedia({ video: true })
        .then(stream => {
          if (videoRef.current) {
            videoRef.current.srcObject = stream;
          }
        })
        .catch(err => console.log(err));
    }
  }, []);

  return (
    <div style={{display: "flex", width: "100%", height: "100%"}}>
      <video ref={videoRef} autoPlay></video>
    </div>
  );
}

export default Camera;
