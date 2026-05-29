const char P_DOOR_PAGE[] PROGMEM = R"rawliteral(
 <!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Car Control</title>
  <style>
    body {
      margin: 0;
      padding: 0;
      font-family: sans-serif;
      background: #f5f5f5;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
    }

    .container {
      text-align: center;
      padding: 20px;
    }

    h1 {
      margin-bottom: 30px;
      font-size: 1.5rem;
      color: #333;
    }

    button {
      width: 150px;
      padding: 12px;
      margin: 10px;
      font-size: 1rem;
      border: none;
      border-radius: 6px;
      background-color: #007bff;
      color: white;
      cursor: pointer;
    }

    button.lock {
      background-color: #dc3545;
    }

    #status {
      margin-top: 20px;
      font-size: 0.9rem;
      color: #555;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Car Control</h1>
    <button class="lock" onclick="sendCommand('lock')">Lock</button>
    <button onclick="sendCommand('unlock')">Unlock</button>
    <div id="status">Status: Ready</div>
  </div>

  <script>
    function sendCommand(cmd) {
      fetch('/door_' + cmd )
        .then(res => res.text())
        .then(text => {
          document.getElementById('status').innerText = 'Status: ' + text;
        })
        .catch(() => {
          document.getElementById('status').innerText = 'Status: Error';
        });
    }
  </script>
</body>
</html>
)rawliteral";