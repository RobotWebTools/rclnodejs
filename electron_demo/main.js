// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

const { app, BrowserWindow } = require('electron')
let rclnodejs = require("rclnodejs");
const { ipcMain } = require('electron');

function createWindow() {
  // Create the browser window.
  const mainWindow = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      // Add the following two lines in order to use require() in renderer, see details
      // https://stackoverflow.com/questions/44391448/electron-require-is-not-defined
      nodeIntegration: true,
      contextIsolation: false,
    }
  });

  mainWindow.loadFile('index.html');
}

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.whenReady().then(() => {
  createWindow();
  rclnodejs.init().then(() => {
    let sender = null;
    const node = rclnodejs.createNode('publisher_example_node');
    node.createSubscription('std_msgs/msg/String', 'topic', (msg) => {
      if (sender) {
        sender.send('topic-received', msg.data);
      }
    });
    const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
    ipcMain.on('publish-topic', (event, topic) => {
      publisher.publish(topic);
      sender = event.sender;
    });
    rclnodejs.spin(node);
  });

  app.on('activate', function () {
    // On macOS it's common to re-create a window in the app when the
    // dock icon is clicked and there are no other windows open.
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

// Quit when all windows are closed, except on macOS. There, it's common
// for applications and their menu bar to stay active until the user quits
// explicitly with Cmd + Q.
app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') app.quit()
});
