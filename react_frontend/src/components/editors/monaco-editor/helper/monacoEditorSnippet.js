import {
  snippetsBuilderV2,
  snippetKind
} from "./../index";

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

      // Add basic snippets only if not prevWord
      if (prevWord.word === "") {
        var snippets = snippetsBuilderV2(
          "basic_snippets",
          monaco,
          range,
          "",
        );
      } else {
        var snippets = []
      }

      // Snippets for HAL and GUI
      if (prevWord.word === "GUI" || prevWord.word === "HAL") {
        const suggestions = snippetsBuilderV2(
          "hal_gui",
          monaco,
          range,
          prevWord.word,
        );

        return { suggestions }; 
      }

      // Check if the Robotics Backend is connected
      // Call the RAM for autocompletion
      try {
        window.RoboticsExerciseComponents.commsManager.code_autocomplete({
          code: model.getValue(),
          line: position.lineNumber,
          col: word.endColumn - 1,
        });
      } catch (error) {
        return snippets;
      }

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
      
      return {suggestions: snippets}
    },
  });
};
