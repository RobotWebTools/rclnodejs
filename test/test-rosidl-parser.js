// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import sinon from 'sinon';
import parser from '../rosidl_parser/rosidl_parser.cjs';
import pyUtils from '../rosidl_parser/py_utils.cjs';

const parserDirectory = fileURLToPath(
  new URL('../rosidl_parser/', import.meta.url)
);
const message = `int64 MIN_INT64=-9223372036854775808
uint64 MAX_UINT64=18446744073709551615
int64 safe 9007199254740991
int64 negative_safe -9007199254740991
int64 positive_edge 9007199254740992
int64 negative_edge -9007199254740992
int64 minimum -9223372036854775808
uint64 maximum 18446744073709551615
int64[] signed_values [0, -9223372036854775808, 9223372036854775807, 9007199254740993]
uint64[2] unsigned_values [0, 18446744073709551615]
float64 decimal 1.2345678901234567
float64 scientific 1.25e+30
float64 tiny 1.25e-30
float64[] decimals [1.2345678901234567, 1.25e+30, 1.25e-30]
bool enabled true
string numeric_text "9223372036854775807"
`;

function idlMessage(name) {
  return `
module ${name}_Constants {
  const int64 ${name.toUpperCase()}_MIN_INT64 = -9223372036854775808;
  const uint64 ${name.toUpperCase()}_MAX_UINT64 = 18446744073709551615;
};
typedef uint64 ${name}_Unsigned[2];
struct ${name} {
  @default(value=9007199254740991) int64 safe;
  @default(value=-9007199254740991) int64 negative_safe;
  @default(value=9007199254740992) int64 positive_edge;
  @default(value=-9007199254740992) int64 negative_edge;
  @default(value=-9223372036854775808) int64 minimum;
  @default(value=18446744073709551615) uint64 maximum;
  @default(value="(0, -9223372036854775808, 9223372036854775807, 9007199254740993)")
  sequence<int64> signed_values;
  @default(value="(0, 18446744073709551615)") ${name}_Unsigned unsigned_values;
  @default(value=1.2345678901234567) double decimal;
  @default(value=1.25e+30) double scientific;
  @default(value=1.25e-30) double tiny;
  @default(value="(1.2345678901234567, 1.25e+30, 1.25e-30)")
  sequence<double> decimals;
  @default(value=TRUE) boolean enabled;
  @default(value="9223372036854775807") string numeric_text;
};
`;
}

function checkMessage(spec) {
  const fields = Object.fromEntries(
    spec.fields.map((field) => [field.name, field.default_value])
  );
  assert.deepEqual(fields, {
    safe: Number.MAX_SAFE_INTEGER,
    negative_safe: Number.MIN_SAFE_INTEGER,
    positive_edge: '9007199254740992',
    negative_edge: '-9007199254740992',
    minimum: '-9223372036854775808',
    maximum: '18446744073709551615',
    signed_values: [
      0,
      '-9223372036854775808',
      '9223372036854775807',
      '9007199254740993',
    ],
    unsigned_values: [0, '18446744073709551615'],
    decimal: 1.2345678901234567,
    scientific: 1.25e30,
    tiny: 1.25e-30,
    decimals: [1.2345678901234567, 1.25e30, 1.25e-30],
    enabled: true,
    numeric_text: '9223372036854775807',
  });
  assert.deepEqual(
    spec.constants.map(({ type, value }) => ({ type, value })),
    [
      { type: 'int64', value: '-9223372036854775808' },
      { type: 'uint64', value: '18446744073709551615' },
    ]
  );
}

describe('ROS interface parser JSON precision', function () {
  this.timeout(30000);
  let directory;
  const kinds = [
    { extension: 'msg', type: 'message', sections: [''] },
    { extension: 'srv', type: 'service', sections: ['Request', 'Response'] },
    {
      extension: 'action',
      type: 'action',
      sections: ['Goal', 'Result', 'Feedback'],
    },
  ];

  before(function () {
    directory = fs.mkdtempSync(path.join(os.tmpdir(), 'rclnodejs-parser-'));
    for (const { extension, sections } of kinds) {
      const folder = path.join(directory, extension);
      fs.mkdirSync(folder);
      fs.writeFileSync(
        path.join(folder, `Precision.${extension}`),
        sections.map(() => message).join('---\n')
      );
      fs.writeFileSync(
        path.join(folder, 'Precision.idl'),
        `module precision_msgs { module ${extension} {
${sections.map((name) => idlMessage(name ? `Precision_${name}` : 'Precision')).join('\n')}
}; };`
      );
    }
  });

  after(function () {
    if (directory) fs.rmSync(directory, { recursive: true, force: true });
  });

  it('normalizes only unsafe integers recursively without mutating the input', function () {
    const [executable, args] = pyUtils.getPythonExecutable();
    const json = execFileSync(
      executable,
      [
        ...args,
        '-c',
        `
import sys
sys.path.insert(0, sys.argv[1])
from json_utils import dumps_for_javascript
value = {
    'nested': [{'integers': (0, 2**53 - 1, 2**53, 2**53 + 1, -2**53)}],
    'booleans': [True, False],
    'empty': None,
    'text': '{"value":9223372036854775807}',
    'floats': [1.2345678901234567, 1.25e30, 1.25e-30, -0.0],
}
result = dumps_for_javascript(value)
assert type(value['nested'][0]['integers']) is tuple
assert type(value['nested'][0]['integers'][2]) is int
print(result)
`,
        parserDirectory,
      ],
      { encoding: 'utf8', timeout: 10000 }
    );
    assert.deepEqual(parser._parseJSONObject(json), {
      nested: [
        {
          integers: [
            0,
            Number.MAX_SAFE_INTEGER,
            '9007199254740992',
            '9007199254740993',
            '-9007199254740992',
          ],
        },
      ],
      booleans: [true, false],
      empty: null,
      text: '{"value":9223372036854775807}',
      floats: [1.2345678901234567, 1.25e30, 1.25e-30, -0],
    });
  });

  for (const { extension, type, sections } of kinds) {
    function checkSpec(spec) {
      for (const section of sections) {
        checkMessage(section ? spec[section.toLowerCase()] : spec);
      }
    }

    it(`preserves .${extension} defaults and constants`, async function () {
      const filename = path.join(
        directory,
        extension,
        `Precision.${extension}`
      );
      let spec;
      if (extension === 'msg') {
        spec = await parser.parseMessageFile('precision_msgs', filename);
      } else if (extension === 'srv') {
        spec = await parser.parseServiceFile('precision_msgs', filename);
      } else {
        spec = await parser.parseActionFile('precision_msgs', filename);
      }
      checkSpec(spec);
    });

    it(`preserves IDL ${type} defaults and constants`, async function () {
      const result = await parser.parseIdlFile(
        path.join(directory, extension, 'Precision.idl')
      );
      assert.equal(result.type, type);
      checkSpec(result.spec);
    });
  }

  it('rejects invalid JSON', function () {
    assert.throws(() => parser._parseJSONObject('{"value":'), SyntaxError);
  });

  it('rejects JSON parsing failures through both asynchronous entry points', async function () {
    const error = new SyntaxError('Invalid Python parser output');
    const stub = sinon.stub(parser, '_parseJSONObject').throws(error);
    try {
      await assert.rejects(
        parser.parseMessageFile(
          'precision_msgs',
          path.join(directory, 'msg/Precision.msg')
        ),
        error
      );
      await assert.rejects(
        parser.parseIdlFile(path.join(directory, 'msg/Precision.idl')),
        error
      );
    } finally {
      stub.restore();
    }
  });
});
