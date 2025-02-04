import {
  listed_python_packages,
  snippetsBuilderV2,
  getEditorVariables,
  getEditorFunctions,
  extractClassesAndMembers,
  findClassNameByInstance,
  extractPythonImports,
  snippetKind
} from "./../index";

import { useEffect } from "react";

// Main Editor Snippets
export const monacoEditorSnippet = ({ monaco }) => {
  monaco.languages.register({ id: "python" });

  const EventEmitter = require('events');

  const bus = new EventEmitter();
  let lock = true;

  // Register a completion item provider for the new language
  monaco.languages.registerCompletionItemProvider("python", {
    triggerCharacters: [".","("],
    provideCompletionItems: async (model, position) => {
      lock = true;

      var word = model.getWordUntilPosition(position);
      var prevWord = model.getWordUntilPosition({lineNumber: position.lineNumber, column: position.column - 1});

      var range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };

      if (prevWord.word === "GUI" || prevWord.word === "HAL") {
        var type = prevWord.word;
        const suggestions = snippetsBuilderV2(
          "hal_gui",
          monaco,
          range,
          type,
        );

        console.log(suggestions)

        return { suggestions }; 
      }

      window.RoboticsExerciseComponents.commsManager.code_autocomplete({
        code: model.getValue(),
        line: position.lineNumber,
        col: word.endColumn - 1,
      });

      var snippets = snippetsBuilderV2(
        "basic_snippets",
        monaco,
        range,
        "",
      );;

      const callback = (message) => {
        const data = message.data;

        if (!data) return;

        const new_completions = data.completions;

        new_completions.forEach(snippet => {
          snippets.push({
            label: snippet.label,
            kind: snippetKind({ kind: snippet.type, monaco }),
            detail: snippet.detail,
            documentation: snippet.docstring,
            insertText: snippet.name_with_symbols,
            insertTextRules:
              monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
            range: range,
          });
        });

        lock = false
        bus.emit('unlocked');
      };

      window.RoboticsExerciseComponents.commsManager.suscribreOnce(
        [
          window.RoboticsExerciseComponents.commsManager.events
            .CODE_AUTOCOMPLETE,
        ],
        callback
      );

      if (lock) await new Promise(resolve => bus.once('unlocked', resolve));
      
      // const suggestions = snippetsBuilderV2(
      //   "import",
      //   monaco,
      //   range,
      //   prevWord,
      // );

      // const text = model.getValue();

      // // import extract
      // const allLines = text.split("\n").filter(Boolean);
      // const allImports = [];
      // allLines.forEach((line) => {
      //   const importData = extractPythonImports(line);

      //   if (importData.length) {
      //     if (listed_python_packages.includes(importData[0].importName)) {
      //       allImports.push(importData[0]);
      //     }
      //   }
      // });

      // // check valid import
      // if (allImports.length) {
      //   const importMatch = allImports.find((imp) => {
      //     const match = textUntilPosition.match(
      //       new RegExp(`(${imp.alias})\\.$`)
      //     );
      //     if (match) return imp;
      //   });
      //   console.log(importMatch)
      // }

      
      return {suggestions: snippets}


      //   // match with listed import
      //   if (importMatch) {
      //     const { importName } = importMatch;

      //     // HAL & GUI
      //     if (importName === "GUI" || importName === "HAL") {
      //     } else {
      //       // Other Imports (ex: Numpy, Math)
      //       const suggestions = snippetsBuilderV2({
      //         snippetName: "import",
      //         monaco,
      //         range,
      //         importName,
      //       });

      //       return { suggestions };
      //     }
      //   }
      // }
    },
  });
};
