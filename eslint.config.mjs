import typescriptEslint from "@typescript-eslint/eslint-plugin";
import prettier from "eslint-plugin-prettier";
import globals from "globals";
import tsParser from "@typescript-eslint/parser";
import eslintPluginPrettierRecommended from 'eslint-plugin-prettier/recommended';
import js from "@eslint/js";

export default [
    {
        ignores: [
            "eslint.config.mjs",
            "types/interfaces.d.ts",
            "test/types/index.test-d.ts",
            "**/generated/",
            "**/scripts/",
            "**/benchmark/",
            "**/docs/",
            "**/demo/electron/",
            "**/coverage/",
            "**/dist/",
            "**/prebuilds/",
            "**/build/",
        ],
    },
    {
        ...js.configs.recommended,
        files: ["lib/**/*.js", "index.js"],
    },
    {
        rules: {
            'no-dupe-class-members': 'off',
        },
    },
    {
        plugins: {
            "@typescript-eslint": typescriptEslint,
        },
        languageOptions: {
            globals: {
                ...globals.node,
            },
            parser: tsParser,
            ecmaVersion: "latest",
            sourceType: "module",
        },
        files: ['types/*.d.ts'],
        rules: {
            ...typescriptEslint.configs.recommended.rules,
            "@typescript-eslint/no-explicit-any": "off",
            "@typescript-eslint/triple-slash-reference": "off",
        },
    },
    {
        plugins: {
            prettier,
        },
        languageOptions: {
            globals: {
                ...globals.node,
            },
            ecmaVersion: "latest",
            sourceType: "module",
        },
        files: ["lib/**/*.js", "test/**/*.js", "bin/**/*.js", "rosocket/**/*.js", "index.js", "example/**/*.mjs"],
        rules: {
            ...eslintPluginPrettierRecommended.rules,
        },
    },
    {
        plugins: {
            prettier,
        },
        languageOptions: {
            globals: {
                ...globals.node,
            },
            ecmaVersion: "latest",
            sourceType: "commonjs",
        },
        files: ["rosidl_parser/**/*.{js,cjs}", "rosidl_gen/**/*.{js,cjs}",
            "rostsd_gen/**/*.{js,cjs}", "example/**/*.cjs"],
        rules: {
            ...eslintPluginPrettierRecommended.rules,
        },
    }
];
