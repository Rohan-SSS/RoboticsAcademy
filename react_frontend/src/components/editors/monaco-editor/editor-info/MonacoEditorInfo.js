import React from "react";
import {
  AlignIcon,
  CloseIcon,
  InfoIcon,
  KeyboardIcon,
  WidgetsIcon,
  ZoomInIcon,
  ZoomOutIcon,
} from "../icons";

const MonacoEditorInfo = ({ editorSettings, dispatch, editorRef }) => {
  const { isModalOpen, isCodeFormatEnable, isZoomingEnable, modalScreenState } =
    editorSettings;

  const ctrlSEvent = new KeyboardEvent("keydown", {
    key: "s",
    ctrlKey: true,
    bubbles: true,
  });

  const handleFormatCode = () => {
    editorRef.current.getDomNode().dispatchEvent(ctrlSEvent);
  };

  const handleFontZoom = (zoom) => {
    const currentFontSize = editorRef.current.getOption(
      monaco.editor.EditorOption.fontSize
    );

    // font size between 10 and 100
    if (zoom === "up") {
      editorRef.current.updateOptions({
        fontSize: Math.min(100, currentFontSize + 1),
      });
    } else {
      editorRef.current.updateOptions({
        fontSize: Math.max(10, currentFontSize - 1),
      });
    }
  };

  return (
    <>
      {isModalOpen && (
        <>
          {/* blur layer */}
          <div
            className="w-full h-full absolute left-0 right-0 z-[99] "
            style={{
              background: "rgba(0, 0, 0, 0.02)",
              filter: "blur(2px)",
              backdropFilter: "blur(2px)",
            }}
          ></div>
          {/* modal */}
          <div
            className="flex absolute w-[400px] h-[250px] left-[50%] top-[50%] bg-[#2D2D2D]  rounded-xl -translate-x-[50%] -translate-y-[50%] z-[100]"
            style={{ border: "1px solid #464646" }}
          >
            {/* sizebar */}
            <div className="flex flex-col items-center justify-start pt-10 gap-2 w-[120px] h-full bg-[#383838] rounded-xl">
              {/* shortcuts */}
              <div
                className={`flex items-center justify-center  gap-1 w-[100px] h-7 rounded-full cursor-pointer ${
                  modalScreenState === "shortcuts" ? `bg-[#FFA726]` : ``
                }`}
                onClick={() =>
                  dispatch({
                    type: "changeModalScreenState",
                    payload: { screen: "shortcuts" },
                  })
                }
              >
                <KeyboardIcon cssClass="" />
                <span className="text-[#fcfcfc] text-sm">shortcuts</span>
              </div>
              {/* widgets */}
              <div
                className={`flex items-center justify-center  gap-1 w-[100px] h-7 rounded-full cursor-pointer ${
                  modalScreenState === "widgets" ? `bg-[#FFA726]` : ``
                }`}
                onClick={() =>
                  dispatch({
                    type: "changeModalScreenState",
                    payload: { screen: "widgets" },
                  })
                }
              >
                <WidgetsIcon cssClass="" />
                <span className="text-[#fcfcfc] text-sm">widgets</span>
              </div>
            </div>
            {/* main */}
            <div className="flex  flex-col w-[calc(400px-120px)] h-full ">
              {/* close button */}
              <div
                className="absolute top-1 right-1 flex justify-center items-center w-7 h-7 hover:bg-[#393939] rounded-full duration-100 cursor-pointer"
                onClick={() =>
                  dispatch({
                    type: "changeSettingsModalState",
                    payload: { isModalOpen: !isModalOpen },
                  })
                }
              >
                <CloseIcon cssClass="" />
              </div>
              {/* BODY */}
              <div className={`w-full h-full pt-10 items-center px-6`}>
                {/* shortcuts */}
                {modalScreenState === "shortcuts" && (
                  <div className="w-full flex flex-col items-start gap-5">
                    {/*  */}
                    <div className="w-full flex items-start justify-between text-sm text-[#fcfcfc]">
                      <div className="">Code Format</div>
                      <div className="flex items-center gap-1">
                        <span className="bg-[#383838] px-2 py-[2px] rounded-md border-[1px] border-[#636363]">
                          ctrl
                        </span>
                        +
                        <span className="bg-[#383838] px-2 py-[2px] rounded-md border-[1px] border-[#636363]">
                          s
                        </span>
                      </div>
                    </div>
                    {/*  */}
                    <div className="w-full flex items-center justify-between text-sm text-[#fcfcfc]">
                      <div className="">Font Size</div>
                      <div className="flex items-center gap-1">
                        <span className="bg-[#383838] px-2 py-[2px] rounded-md border-[1px] border-[#636363]">
                          ctrl
                        </span>
                        +
                        <span className="bg-[#383838] px-2 py-[2px] rounded-md border-[1px] border-[#636363]">
                          wheel
                        </span>
                      </div>
                    </div>
                  </div>
                )}
                {/* widgets */}
                {modalScreenState === "widgets" && <div>widgets</div>}
              </div>
            </div>
          </div>
        </>
      )}
      {/* buttons */}
      <div
        className={`absolute bottom-2 right-2 flex justify-between items-center gap-1 h-9 z-[100] bg-[#2D2D2D] rounded-full border-[#464646] border-[1px] ${
          isCodeFormatEnable || isZoomingEnable ? `px-[4px]` : `px-[2px]`
        }`}
      >
        {/* font size */}
        {isZoomingEnable && (
          <div className="flex items-center gap-1">
            <div className="flex justify-center items-center  h-7 rounded-full cursor-pointer gap-0.5">
              {/* zoom in */}
              <div
                className="flex justify-center items-center  w-[30px] h-7 bg-[#474747] rounded-tl-full rounded-bl-full duration-100 "
                onClick={() => handleFontZoom("up")}
              >
                <ZoomInIcon cssClass="" />
              </div>
              {/* zoom out */}
              <div
                className="flex justify-center items-center  w-[30px] h-7 bg-[#474747] rounded-tr-full rounded-br-full duration-100 "
                onClick={() => handleFontZoom("down")}
              >
                <ZoomOutIcon cssClass="" />
              </div>
            </div>
            <div className="h-6 w-[2px] bg-[#464646]"></div>
          </div>
        )}
        {/* code format icon */}
        {isCodeFormatEnable && (
          <div className="flex items-center gap-1">
            <div
              className="flex justify-center items-center w-7 h-7 rounded-full hover:bg-[#474747] duration-100 cursor-pointer"
              onClick={() => handleFormatCode()}
            >
              <AlignIcon cssClass="" />
            </div>

            <div className="h-6 w-[2px] bg-[#464646]"></div>
          </div>
        )}
        {/* question mark */}
        <div
          className="flex justify-center items-center w-7 h-7 rounded-full hover:bg-[#474747] duration-100 cursor-pointer"
          onClick={() =>
            dispatch({
              type: "changeSettingsModalState",
              payload: { isModalOpen: !isModalOpen },
            })
          }
        >
          <InfoIcon cssClass="" />
        </div>
      </div>
    </>
  );
};

export default MonacoEditorInfo;

/*



background: rgba(0, 0, 0, 0.002);
filter: blur(3px);
backdrop-filter: blur(3px);


*/
