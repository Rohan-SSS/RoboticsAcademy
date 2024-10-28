import * as React from "react";
import { Box } from "@mui/material";

import "../../styles/editors/EditorRobot.css";

// monaco editor import start
import "../../styles/tailwind.css";
import {
  MonacoEditor,
  defaultEditorSourceCode,
} from "./monaco-editor";
import { useEditorReudcer } from "../../hooks/useEditorReudcer";
// monaco editor import end

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

window.RoboticsReactComponents.CodeEditor = (function () {
  let editorCode = "";
  const editorCodeChangeSuscribers = [];

  const setCode = (code) => {
    editorCode = code;
    for (
      let i = 0, length = editorCodeChangeSuscribers.length;
      i < length;
      ++i
    ) {
      editorCodeChangeSuscribers[i](code);
    }
  };

  const OnEditorCodeChanged = (handler) => {
    editorCodeChangeSuscribers.push(handler);
  };

  const getCode = () => editorCode;

  return {
    setCode: setCode,
    getCode: getCode,
    OnEditorCodeChanged: OnEditorCodeChanged,
  };
})();

// Rect Components for Monaco
window.RoboticsReactComponentsMonaco =
  window.RoboticsReactComponentsMonaco || {};

window.RoboticsReactComponentsMonaco.CodeEditor = (function () {
  let editorCode = "";
  const editorCodeChangeSuscribers = [];

  //
  let isActive = true;
  const setActive = (active) => (isActive = active);
  const getActive = () => isActive;
  //
  const setCode = (code) => {
    editorCode = code;
    for (
      let i = 0, length = editorCodeChangeSuscribers.length;
      i < length;
      ++i
    ) {
      editorCodeChangeSuscribers[i](code);
    }
  };

  const OnEditorCodeChanged = (handler) => {
    editorCodeChangeSuscribers.push(handler);
  };

  const getCode = () => editorCode;

  return {
    setCode: setCode,
    getCode: getCode,
    OnEditorCodeChanged: OnEditorCodeChanged,
    //
    setActive: setActive,
    getActive: getActive,
  };
})();

export default function EditorRobot(props) {
  const [monacoEditorSourceCode, setMonacoEditorSourceCode] = React.useState(
    defaultEditorSourceCode
  );

  React.useEffect(() => {
    // monaco
    RoboticsReactComponentsMonaco.CodeEditor.setCode(monacoEditorSourceCode);
    RoboticsReactComponentsMonaco.CodeEditor.OnEditorCodeChanged((code) => {
      setMonacoEditorSourceCode(code);
    });
  }, []);

  //! Monaco Code Editor

  const [state, dispatch] = useEditorReudcer();
  React.useEffect(() => {
    RoboticsReactComponentsMonaco.CodeEditor.setActive(true);
  }, [state.activeEditor]);

  // monaco editor code change
  const handleMonacoEditorCodeChange = (code) => {
    setMonacoEditorSourceCode(code);
    RoboticsReactComponentsMonaco.CodeEditor.setCode(code);
  };

  return (
    <Box id="code-container">
      <MonacoEditor
        state={state}
        dispatch={dispatch}
        monacoEditorSourceCode={monacoEditorSourceCode}
        setMonacoEditorSourceCode={setMonacoEditorSourceCode}
        handleMonacoEditorCodeChange={handleMonacoEditorCodeChange}
      />
    </Box>
  );
}
