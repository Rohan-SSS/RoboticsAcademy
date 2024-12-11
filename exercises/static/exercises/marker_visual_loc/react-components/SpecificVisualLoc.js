import * as React from "react";
import PropTypes from "prop-types";
import { drawImage, updatePath, addToPath } from "./helpers/showImageVisual";
import RobotRed from "../resources/images/robot_red.svg";
import RobotGreen from "../resources/images/robot_green.svg";
import RobotBlue from "../resources/images/robot_blue.svg";

import house from "../resources/images/map.png";

import "./css/GUICanvas.css"
function SpecificVisualLoc(props) {
  const [realPose, setRealPose] = React.useState(null)
  const [noisyPose, setNoisyPose] = React.useState(null)
  const [userPose, setUserPose] = React.useState(null)
  const [realPath, setRealPath] = React.useState("")
  const [noisyPath, setNoisyPath] = React.useState("")
  const [userPath, setUserPath] = React.useState("")
  var realTrail = [];
  var noisyTrail = [];
  var userTrail = [];
  var realLastPose = undefined;
  var noisyLastPose = undefined;
  var userLastPose = undefined;

  // TODO: Add shadows of poses

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      const updateData = message.data.update;
      console.log(message);

      var img = document.getElementById('gui-canvas'); 
      //or however you get a handle to the IMG
      var width = (1012 / 300) / (1012 /img.clientWidth);
      var height = (1012 / 150) / (1012 /img.clientHeight);

      if (updateData.real_pose) {
        const pose = updateData.real_pose.substring(1, updateData.real_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));

        updatePath(realTrail, setRealPath, height, width);
        setRealPose([content[1]*height,content[0]*width, -1.57 -content[2]]);
        addToPath(content[1], content[0], realTrail);
      }

      if (updateData.noisy_pose) {
        const pose = updateData.noisy_pose.substring(1, updateData.noisy_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));

        setNoisyPose([content[1]*height,content[0]*width, -1.57 -content[2]]);
      }

      if (updateData.estimate_pose) {
        const pose = updateData.estimate_pose.substring(1, updateData.estimate_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));

        setUserPose([content[1]*height,content[0]*width, -1.57 -content[2]]);
      }

      if (updateData.image) {
        console.log("image");
        drawImage(updateData);
      }

      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
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

  React.useEffect(() => {
    const callback = (message) => {
      console.log(message);
      if (message.data.state === "visualization_ready") {
        try {
          setRealPose(null)
          setNoisyPose(null)
          setUserPose(null)
        } catch (error) {
        }
      }
    }
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, [])

  return (
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative"}}>
      <img src={house} alt="" className="exercise-canvas" id="exercise-img"/>
      <img className="image" id="gui-canvas" style={{left: "50%"}}
        src="https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"/>
      {realPose &&
        <div id="real-pos" style={{rotate: "z "+ realPose[2]+"rad", top: realPose[0] -5 , left: realPose[1] -5}}>
          <img src={RobotGreen} id="real-pos"/>
        </div>
      }
      {realPath &&
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:2, position:"absolute"}}>
          <path xmlns="http://www.w3.org/2000/svg" d={realPath} 
            style={{strokeWidth: "20px", strokeLinejoin:"round", stroke: "green", fill: "none"}}
          />
        </svg>
      }
      {noisyPose &&
        <div id="noisy-pos" style={{rotate: "z "+ noisyPose[2]+"rad", top: noisyPose[0] -5 , left: noisyPose[1] -5}}>
          <img src={RobotBlue} id="noisy-pos"/>
        </div>
      }
      {userPose &&
        <div id="user-pos" style={{rotate: "z "+ userPose[2]+"rad", top: userPose[0] -5 , left: userPose[1] -5}}>
          <img src={RobotRed} id="user-pos"/>
        </div>
      }
    </div>
  );
}

SpecificVisualLoc.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificVisualLoc;