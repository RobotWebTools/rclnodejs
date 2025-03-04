import prettier from "eslint-plugin-prettier";
import globals from "globals";
import typescriptEslint from "@typescript-eslint/eslint-plugin";
import tsParser from "@typescript-eslint/parser";
import path from "node:path";
import { fileURLToPath } from "node:url";
import js from "@eslint/js";
import { FlatCompat } from "@eslint/eslintrc";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const compat = new FlatCompat({
    baseDirectory: __dirname,
    recommendedConfig: js.configs.recommended,
    allConfig: js.configs.all
});

export default [{
    ignores: ["generated/", "eslint.config.mjs", "lib/", "types/", "scripts/", "benchmark/", "docs/", "electron_demo/"],
}, ...compat.extends("prettier"), {
    plugins: {
        prettier,
    },

    languageOptions: {
        globals: {
            ...globals.node,
            every: true,
            after: true,
            constantly: true,
        },

        ecmaVersion: "latest",
        sourceType: "commonjs",
    },

    rules: {
        "prettier/prettier": "error",
    },
}, {
    files: ["types/*.d.ts"],

    plugins: {
        "@typescript-eslint": typescriptEslint,
    },

    languageOptions: {
        parser: tsParser,
        sourceType: "module",
    },

    rules: {
        strict: [0, "global"],
    },
}];
