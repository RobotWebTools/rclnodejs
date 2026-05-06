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
            sourceType: "commonjs",
        },
        files: ["lib/**/*.js", "rosidl_parser/**/*.js", "rosidl_gen/**/*.js",
            "rostsd_gen/**/*.js", "test/**/*.js", "example/**/*.js", "index.js"],
        rules: {
            ...eslintPluginPrettierRecommended.rules,
        },
    }
];
