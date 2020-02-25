// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const rclnodejs = require('../index.js');

const ParameterType = rclnodejs.Parameters.ParameterType;
const Parameter = rclnodejs.Parameters.Parameter;
const ParameterDescriptor = rclnodejs.Parameters.ParameterDescriptor;

async function main() {
  const NODE_NAME = 'my_node';

  // commandline override of param1
  const argv = ['--ros-args', '-p', NODE_NAME + ':param1:=hello ros2'];

  // initialize rclnodejs with commandline argv
  await rclnodejs.init(rclnodejs.Context.defaultContext(), argv);

  const node = rclnodejs.createNode(NODE_NAME);

  // define param
  const parameter = new Parameter(
    'param1',
    ParameterType.PARAMETER_STRING,
    'hello world'
  );
  const parameterDescriptor = new ParameterDescriptor(
    'param1',
    ParameterType.PARAMETER_STRING
  );

  // declare param1
  node.declareParameter(parameter, parameterDescriptor);
  console.log(`Declared parameter: ${parameter.name}`);

  if (!node.hasParameter('param1')) {
    console.error(`Unable to find parameter: ${parameter.name}`);
    return;
  }

  console.log('Parameter overridden: ', node.getParameter('param1'));
  console.log(node.getParameterDescriptor('param1'));
}

main();
